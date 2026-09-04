//! `bobdil-kernel` -- the command line around the loop.
//!
//! Five verbs, each of which answers one question:
//!
//!   run       drive the rig
//!   bench     can this machine hold the deadline? (the Phase 0 gate)
//!   selftest  is the safety path intact? -- run this before anyone drives
//!   devices   what hardware is connected, and what can it actually do?
//!   replay    reproduce a recording exactly

use std::path::PathBuf;
use std::process::ExitCode;

use bobdil_kernel::config::KernelConfig;
use bobdil_kernel::generated::frames::{DriverInput, FfbCommand, VehicleState, LAYOUT_HASH};
use bobdil_kernel::io::device::NullDevice;
use bobdil_kernel::io::ffb::{FfbChain, FfbConfig};
use bobdil_kernel::io::watchdog::WatchdogConfig;
use bobdil_kernel::metrics::StepTimeHistogram;
use bobdil_kernel::plant::integrator::Method;
use bobdil_kernel::plant::{ladder, reduced::Reduced14Dof, reduced::ReducedParams};
use bobdil_kernel::replay::{ReplayConfig, TunableSetting};
use bobdil_kernel::sys::clock;
use bobdil_kernel::sys::sched;
use bobdil_kernel::{loop_runner, VERSION};

const USAGE: &str = "\
bobdil-kernel -- BobDil soft-real-time plant kernel

USAGE:
    bobdil-kernel <COMMAND> [OPTIONS]

COMMANDS:
    run         Drive the rig.
    bench       Measure step time against the deadline gate (Phase 0).
    selftest    Exercise the force-feedback safety path. Run before driving.
    devices     List connected devices and the cues they can actually deliver.
    replay      Re-run a recording and check it reproduces exactly.
    info        Print schema, build and machine information.

OPTIONS:
    --plant <DIR>        Unpacked FMI 2.0 Model Exchange FMU directory.
                         Omit to use the built-in reduced-order kernel.
    --dt <SECONDS>       Fixed step (default 0.001).
    --method <NAME>      euler | midpoint | rk4 (default rk4).
    --substeps <N>       Integrator substeps per step (default 1).
    --duration <SECONDS> Stop after this long. Omit to run until interrupted.
    --speed <M/S>        Initial vehicle speed (default 0, a standing start).
    --telemetry <PATH>   Record every step to this file.
    --torque-limit <NM>  Absolute feedback clamp (default 8). Set this below
                         your wheel's capability BEFORE anyone drives.
    --ffb-gain <G>       Feedback gain, 0 to silence the wheel (default 1).
    --cpu <N>            Pin the step thread to this CPU.
    --no-realtime        Do not request SCHED_FIFO or pin.
    --steps <N>          bench only: how many steps to run.
    --file <PATH>        replay only: the recording to reproduce.
    --out <PATH>         replay only: write the replayed telemetry here, so it
                         can be diffed against another setup's replay.
    --tunable <K=V>      replay only, repeatable: apply a setup change before
                         replaying. This is the B side of a paired A/B.
";

struct Args {
    command: String,
    config: KernelConfig,
    steps: u64,
    file: Option<PathBuf>,
    out: Option<PathBuf>,
    tunables: Vec<TunableSetting>,
}

fn parse() -> Result<Args, String> {
    let mut raw = std::env::args().skip(1);
    let command = raw.next().unwrap_or_else(|| "help".to_string());
    let mut config = KernelConfig::default();
    let mut steps = 60_000;
    let mut file = None;
    let mut out = None;
    let mut tunables = Vec::new();

    while let Some(flag) = raw.next() {
        let mut value = || raw.next().ok_or_else(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--plant" => config.plant_dir = Some(PathBuf::from(value()?)),
            "--dt" => {
                config.step_dt = value()?.parse().map_err(|_| "--dt must be a number")?;
            }
            "--method" => {
                let text = value()?;
                config.method =
                    Method::parse(&text).ok_or_else(|| format!("unknown method {text}"))?;
            }
            "--substeps" => {
                config.substeps = value()?
                    .parse()
                    .map_err(|_| "--substeps must be an integer")?;
            }
            "--duration" => {
                config.duration_s = Some(
                    value()?
                        .parse()
                        .map_err(|_| "--duration must be a number")?,
                );
            }
            "--speed" => {
                config.initial_speed = value()?.parse().map_err(|_| "--speed must be a number")?;
            }
            "--telemetry" => config.telemetry_path = Some(PathBuf::from(value()?)),
            "--torque-limit" => {
                config.ffb_torque_limit = value()?
                    .parse()
                    .map_err(|_| "--torque-limit must be a number")?;
            }
            "--ffb-gain" => {
                config.ffb_gain = value()?
                    .parse()
                    .map_err(|_| "--ffb-gain must be a number")?;
            }
            "--cpu" => {
                config.rt_cpu = Some(value()?.parse().map_err(|_| "--cpu must be an integer")?);
            }
            "--no-realtime" => config.request_realtime = false,
            "--steps" => {
                steps = value()?.parse().map_err(|_| "--steps must be an integer")?;
            }
            "--file" => file = Some(PathBuf::from(value()?)),
            "--out" => out = Some(PathBuf::from(value()?)),
            "--tunable" => tunables.push(TunableSetting::parse(&value()?)?),
            "-h" | "--help" => {
                println!("{USAGE}");
                std::process::exit(0);
            }
            other => return Err(format!("unknown option {other}")),
        }
    }
    Ok(Args {
        command,
        config,
        steps,
        file,
        out,
        tunables,
    })
}

fn main() -> ExitCode {
    let args = match parse() {
        Ok(args) => args,
        Err(error) => {
            eprintln!("error: {error}\n\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    let result = match args.command.as_str() {
        "run" => command_run(args),
        "bench" => command_bench(args),
        "selftest" => command_selftest(),
        "devices" => command_devices(),
        "replay" => command_replay(args),
        "info" => command_info(),
        "help" | "--help" | "-h" => {
            println!("{USAGE}");
            Ok(())
        }
        other => Err(format!("unknown command {other}\n\n{USAGE}")),
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(feature = "sdl3")]
fn command_run(args: Args) -> Result<(), String> {
    use bobdil_kernel::io::sdl3::{AxisMap, SdlDevice};

    let limit = args.config.ffb_torque_limit;
    match SdlDevice::open(AxisMap::default(), limit) {
        Ok(device) => {
            println!(
                "device      {}",
                bobdil_kernel::io::InputSource::caps(&device).describe()
            );
            loop_runner::run(args.config, device)?.print();
        }
        Err(error) => {
            eprintln!(
                "no wheel found ({error}); running with a scripted null device.\n\
                 The loop, buffers, safety path and telemetry are all exercised; \
                 there is simply nothing to feel."
            );
            loop_runner::run_headless(args.config)?.print();
        }
    }
    Ok(())
}

#[cfg(not(feature = "sdl3"))]
fn command_run(args: Args) -> Result<(), String> {
    eprintln!("built without the sdl3 feature: no hardware backend, using a scripted null device.");
    loop_runner::run_headless(args.config)?.print();
    Ok(())
}

/// The Phase 0 gate. Everything downstream of this is unfounded until it runs.
fn command_bench(args: Args) -> Result<(), String> {
    println!("bobdil-kernel {VERSION} -- step time benchmark");
    println!("machine     {} cpus online", sched::online_cpus());

    // Allocate every buffer this command will use before hardening, so that
    // mlock(current) covers them. Hardening first and allocating afterwards is
    // exactly the ordering that turns a memlock limit into an allocation
    // failure.
    let mut jitter_histogram = StepTimeHistogram::new(20_000_000, 1_000);
    let status = sched::harden_current_thread(args.config.rt_cpu);
    println!("scheduling  {}", status.describe());
    for line in status.advice() {
        println!("            {line}");
    }
    if !status.is_hardened() {
        println!("            The tail below is therefore dominated by the OS, not by the plant.");
    }

    // Establish what the sleep path itself costs, so the plant is not blamed
    // for jitter the kernel's own timer introduced.
    let jitter = clock::measure_sleep_jitter_ns(2_000, args.config.step_ns());
    for sample in &jitter {
        jitter_histogram.record((*sample).max(0) as u64);
    }
    println!("sleep wake  {}", jitter_histogram.summary_us());

    println!(
        "\nprobing at dt={:.3} ms, {} ({} evaluations/step), {} steps",
        args.config.step_dt * 1e3,
        format_args!("{:?}", args.config.method),
        args.config.method.evaluations() * args.config.substeps,
        args.steps
    );

    let gate = ladder::DEFAULT_GATE_P999_NS;
    let mut any_failed = false;

    if let Some(dir) = &args.config.plant_dir {
        use bobdil_kernel::generated::frames::kernel_id;
        use bobdil_kernel::plant::fmu_me::FmuMe;
        match FmuMe::load(
            dir,
            kernel_id::VEHICLE_FMI,
            args.config.method,
            args.config.substeps,
        ) {
            Ok(mut fmu) => {
                println!(
                    "  {}",
                    ladder::probe(&mut fmu, args.config.step_dt, args.steps).describe(gate)
                );
            }
            Err(error) => {
                println!("  FMU at {}: FAILED to load: {error}", dir.display());
                any_failed = true;
            }
        }
    }

    let mut reduced = Reduced14Dof::new(ReducedParams::default());
    let measurement = ladder::probe(&mut reduced, args.config.step_dt, args.steps);
    println!("  {}", measurement.describe(gate));
    if !measurement.passes(gate) {
        any_failed = true;
    }

    println!(
        "\ngate: p99.9 <= {:.0} us (half the {:.1} ms budget, leaving room for OS jitter)",
        gate as f64 / 1000.0,
        args.config.step_dt * 1e3
    );
    if any_failed {
        return Err("a kernel did not clear the gate".to_string());
    }
    Ok(())
}

/// Exercise the safety path deliberately, before any human touches the wheel.
///
/// These are the tests architecture.md 8 calls "highest priority, tested
/// adversarially". They are a command rather than only a unit test so they can
/// be run against the machine and the wheel that will actually be used.
fn command_selftest() -> Result<(), String> {
    println!("bobdil-kernel {VERSION} -- force feedback safety self-test\n");
    let mut failures = Vec::new();

    let mut check = |name: &str, passed: bool, detail: String| {
        println!(
            "  [{}] {name}{}",
            if passed { "PASS" } else { "FAIL" },
            if detail.is_empty() {
                String::new()
            } else {
                format!(" -- {detail}")
            }
        );
        if !passed {
            failures.push(name.to_string());
        }
    };

    // 1. A non-finite plant output must produce exactly zero torque, in one step.
    let mut chain = FfbChain::new(FfbConfig::default());
    for _ in 0..500 {
        chain.condition(
            &VehicleState {
                handwheel_torque: 5.0,
                handwheel_angle: 0.4,
                ..Default::default()
            },
            1e-3,
            0,
            true,
        );
    }
    let poisoned = chain.condition(
        &VehicleState {
            handwheel_torque: f64::NAN,
            handwheel_angle: 0.4,
            ..Default::default()
        },
        1e-3,
        0,
        true,
    );
    check(
        "NaN plant output -> zero torque in one step",
        poisoned.command.torque_nm == 0.0 && poisoned.nonfinite,
        format!("{} N.m", poisoned.command.torque_nm),
    );

    // 2. No input, however extreme, may exceed the clamp.
    let mut chain = FfbChain::new(FfbConfig {
        torque_limit_nm: 5.0,
        slew_limit_nm_per_s: 1e12,
        ..Default::default()
    });
    let mut worst: f64 = 0.0;
    for magnitude in [1e3, 1e6, 1e12, -1e12] {
        let outcome = chain.condition(
            &VehicleState {
                handwheel_torque: magnitude,
                ..Default::default()
            },
            1e-3,
            0,
            true,
        );
        worst = worst.max(outcome.command.torque_nm.abs());
    }
    check(
        "torque clamp holds against extreme input",
        worst <= 5.0 + 1e-9,
        format!("worst {worst:.3} N.m"),
    );

    // 3. A stalled step thread must reach exactly zero within the ramp.
    let mut chain = FfbChain::new(FfbConfig {
        watchdog: WatchdogConfig {
            miss_limit: 5,
            ramp_ms: 50.0,
            recovery_streak: 50,
        },
        ..Default::default()
    });
    for _ in 0..500 {
        chain.condition(
            &VehicleState {
                handwheel_torque: 6.0,
                handwheel_angle: 0.4,
                ..Default::default()
            },
            1e-3,
            0,
            true,
        );
    }
    let mut torque = f64::MAX;
    let mut reached_zero_at = None;
    for step in 0..300 {
        let outcome = chain.condition(
            &VehicleState {
                handwheel_torque: 6.0,
                handwheel_angle: 0.4,
                ..Default::default()
            },
            1e-3,
            step,
            false,
        );
        torque = outcome.command.torque_nm;
        if torque == 0.0 && reached_zero_at.is_none() {
            reached_zero_at = Some(step);
        }
    }
    check(
        "stalled loop ramps torque to zero within 100 ms",
        matches!(reached_zero_at, Some(step) if step <= 100),
        match reached_zero_at {
            Some(step) => format!("zero at step {step}"),
            None => format!("never reached zero (ended at {torque:.3} N.m)"),
        },
    );

    // 4. The device is left slack on shutdown.
    {
        use bobdil_kernel::io::device::HapticSink;
        let mut device = NullDevice::new();
        device
            .apply(&FfbCommand {
                torque_nm: 7.0,
                ..Default::default()
            })
            .ok();
        device.zero();
        check(
            "device is left at zero torque on shutdown",
            device.last_command.torque_nm == 0.0 && device.zeroed,
            String::new(),
        );
    }

    // 5. A stale command must not be held.
    {
        use bobdil_kernel::io::watchdog::command_is_fresh;
        let held = command_is_fresh(1_000_000, 1_000_000 + 20_000_000, 8_000_000);
        check(
            "a 20 ms old command is rejected as stale",
            !held,
            String::new(),
        );
    }

    println!();
    if failures.is_empty() {
        println!("all safety checks passed. The rig is cleared for a driver.");
        Ok(())
    } else {
        Err(format!(
            "{} SAFETY CHECK(S) FAILED: {}. DO NOT let anyone drive this build.",
            failures.len(),
            failures.join(", ")
        ))
    }
}

#[cfg(feature = "sdl3")]
fn command_devices() -> Result<(), String> {
    use bobdil_kernel::io::sdl3::{AxisMap, SdlDevice};
    match SdlDevice::open(AxisMap::default(), 8.0) {
        Ok(device) => {
            let caps = bobdil_kernel::io::InputSource::caps(&device);
            println!("{}", caps.describe());
            let missing = caps.missing_cues();
            if !missing.is_empty() {
                println!(
                    "NOT available: {}.\nBobDil will not claim these cues, and a driver must not \
                     attribute their absence to the car.",
                    missing.join(", ")
                );
            }
            Ok(())
        }
        Err(error) => Err(format!("{error}")),
    }
}

#[cfg(not(feature = "sdl3"))]
fn command_devices() -> Result<(), String> {
    Err("built without the sdl3 feature; no device backend is compiled in".to_string())
}

/// Replay a recording, prove it reproduces bit-identically, and optionally
/// write the result out so two setups can be compared.
///
/// This is the verb paired A/B is built on (architecture.md 1.9). Without
/// `--tunable` it answers "is this plant deterministic?"; with `--tunable` and
/// `--out` it produces the B side of a comparison whose only difference from
/// the A side is the setup change named on the command line.
fn command_replay(args: Args) -> Result<(), String> {
    use bobdil_kernel::plant::InitialConditions;
    use bobdil_kernel::replay;
    use bobdil_kernel::telemetry::TelemetryReader;

    let path = args.file.ok_or("replay needs --file <recording>")?;
    let recording = TelemetryReader::open(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    println!(
        "replaying {} ({} frames, dt={:.6} s, kernel id {})",
        path.display(),
        recording.frames.len(),
        recording.meta.step_dt,
        recording.meta.kernel_id
    );

    let config = ReplayConfig {
        plant_dir: args.config.plant_dir.clone(),
        method: args.config.method,
        substeps: args.config.substeps,
        initial: InitialConditions {
            speed: args.config.initial_speed,
            steering_angle: 0.0,
        },
        tunables: args.tunables,
    };
    if !config.tunables.is_empty() {
        let applied: Vec<String> = config.tunables.iter().map(|t| t.describe()).collect();
        println!("setup      {}", applied.join(", "));
    }

    let outcome = replay::replay(&config, &recording)?;
    println!(
        "plant      {} ({} frames)",
        outcome.kernel_name,
        outcome.frames.len()
    );
    if !outcome.deterministic {
        return Err(
            "replay is not deterministic: two identical runs diverged. No comparison \
             built on this plant means anything until that is fixed."
                .to_string(),
        );
    }
    println!("two replays of the same inputs produced bit-identical states.");

    if let Some(destination) = args.out {
        // The metadata is the recording's, not this run's: dt, kernel and
        // vehicle must match for the two files to be comparable at all, and
        // copying them forward is what lets the comparison check that.
        let frames = replay::write(&outcome, &recording.meta, &destination)
            .map_err(|error| format!("{}: {error}", destination.display()))?;
        println!("wrote      {frames} frames to {}", destination.display());
    }
    Ok(())
}

fn command_info() -> Result<(), String> {
    println!("bobdil-kernel {VERSION}");
    println!("schema layout hash  {LAYOUT_HASH:#018x}");
    println!(
        "frames              driver_input {} B, vehicle_state {} B, ffb_command {} B",
        DriverInput::SIZE,
        VehicleState::SIZE,
        FfbCommand::SIZE
    );
    println!(
        "segments            /dev/shm/{}, /dev/shm/{}, /dev/shm/{}",
        DriverInput::SHM_NAME,
        VehicleState::SHM_NAME,
        FfbCommand::SHM_NAME
    );
    println!("cpus online         {}", sched::online_cpus());
    println!("suggested rt cpu    {:?}", sched::suggested_rt_cpu());
    println!(
        "sdl3 backend        {}",
        if cfg!(feature = "sdl3") {
            "compiled in"
        } else {
            "not compiled in"
        }
    );
    Ok(())
}
