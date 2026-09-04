//! Fixed-step explicit integrators.
//!
//! The host owns the integrator; that is the entire reason BobDil uses FMI
//! Model Exchange rather than Co-Simulation (architecture.md 1.4). A
//! Co-Simulation FMU runs its own variable-step solver inside `fmi2DoStep`,
//! which is unbounded work per call and cannot be given a deadline. Here, a
//! step costs a fixed, known number of derivative evaluations, every time.
//!
//! Everything is preallocated at construction. There is no allocation, and no
//! branch on problem size, inside `advance`.

/// A first-order ODE system the integrator can advance.
pub trait Derivatives {
    /// Write dx/dt at (`t`, `x`) into `dx`.
    fn derivatives(&mut self, t: f64, x: &[f64], dx: &mut [f64]);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    /// One evaluation per step. Cheapest, and the fallback when a step must be
    /// made to fit a shrinking budget.
    Euler,
    /// Two evaluations. Often the right trade for a 1 kHz vehicle model.
    Midpoint,
    /// Four evaluations. The baseline: at 1 ms it carries a comfortable margin
    /// on every eigenvalue in this class of model.
    Rk4,
}

impl Method {
    pub fn evaluations(&self) -> usize {
        match self {
            Self::Euler => 1,
            Self::Midpoint => 2,
            Self::Rk4 => 4,
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text.to_ascii_lowercase().as_str() {
            "euler" => Some(Self::Euler),
            "midpoint" | "rk2" => Some(Self::Midpoint),
            "rk4" => Some(Self::Rk4),
            _ => None,
        }
    }
}

/// A fixed-step integrator with all working storage preallocated.
pub struct FixedStep {
    method: Method,
    /// Substeps taken per call. Lets a fast subsystem be integrated at a
    /// smaller internal step than the loop rate, without changing the rate the
    /// driver and the view see.
    substeps: usize,
    k1: Vec<f64>,
    k2: Vec<f64>,
    k3: Vec<f64>,
    k4: Vec<f64>,
    scratch: Vec<f64>,
}

impl FixedStep {
    pub fn new(dimension: usize, method: Method, substeps: usize) -> Self {
        let substeps = substeps.max(1);
        Self {
            method,
            substeps,
            k1: vec![0.0; dimension],
            k2: vec![0.0; dimension],
            k3: vec![0.0; dimension],
            k4: vec![0.0; dimension],
            scratch: vec![0.0; dimension],
        }
    }

    /// An empty integrator, used only as a temporary stand-in while a plant
    /// hands itself to its own integrator. Allocation-free: a zero-length Vec
    /// never touches the allocator, so this costs nothing on the hot path.
    pub fn placeholder() -> Self {
        Self::new(0, Method::Euler, 1)
    }

    pub fn method(&self) -> Method {
        self.method
    }

    pub fn substeps(&self) -> usize {
        self.substeps
    }

    /// Derivative evaluations one `advance` call will cost. Constant, which is
    /// what makes the step budget predictable.
    pub fn evaluations_per_step(&self) -> usize {
        self.method.evaluations() * self.substeps
    }

    /// Advance `x` from `t` by `dt`. Returns the new time.
    pub fn advance<S: Derivatives + ?Sized>(
        &mut self,
        system: &mut S,
        t: f64,
        x: &mut [f64],
        dt: f64,
    ) -> f64 {
        let h = dt / self.substeps as f64;
        let mut time = t;
        for _ in 0..self.substeps {
            match self.method {
                Method::Euler => self.euler(system, time, x, h),
                Method::Midpoint => self.midpoint(system, time, x, h),
                Method::Rk4 => self.rk4(system, time, x, h),
            }
            time += h;
        }
        time
    }

    fn euler<S: Derivatives + ?Sized>(&mut self, system: &mut S, t: f64, x: &mut [f64], h: f64) {
        system.derivatives(t, x, &mut self.k1);
        for (xi, k1) in x.iter_mut().zip(&self.k1) {
            *xi += h * k1;
        }
    }

    fn midpoint<S: Derivatives + ?Sized>(&mut self, system: &mut S, t: f64, x: &mut [f64], h: f64) {
        system.derivatives(t, x, &mut self.k1);
        Self::stage(&mut self.scratch, x, &self.k1, 0.5 * h);
        system.derivatives(t + 0.5 * h, &self.scratch, &mut self.k2);
        for (xi, k2) in x.iter_mut().zip(&self.k2) {
            *xi += h * k2;
        }
    }

    fn rk4<S: Derivatives + ?Sized>(&mut self, system: &mut S, t: f64, x: &mut [f64], h: f64) {
        system.derivatives(t, x, &mut self.k1);
        Self::stage(&mut self.scratch, x, &self.k1, 0.5 * h);
        system.derivatives(t + 0.5 * h, &self.scratch, &mut self.k2);
        Self::stage(&mut self.scratch, x, &self.k2, 0.5 * h);
        system.derivatives(t + 0.5 * h, &self.scratch, &mut self.k3);
        Self::stage(&mut self.scratch, x, &self.k3, h);
        system.derivatives(t + h, &self.scratch, &mut self.k4);

        let sixth = h / 6.0;
        for (i, xi) in x.iter_mut().enumerate() {
            *xi += sixth * (self.k1[i] + 2.0 * self.k2[i] + 2.0 * self.k3[i] + self.k4[i]);
        }
    }

    /// `out = x + weight * k`, the trial state every explicit stage needs.
    ///
    /// Iterating in lockstep rather than by index is not cosmetic here: this is
    /// the innermost loop in the whole program, and zipping lets the compiler
    /// drop the per-element bounds check it cannot elide when three slices are
    /// indexed independently.
    fn stage(out: &mut [f64], x: &[f64], k: &[f64], weight: f64) {
        for (o, (xi, ki)) in out.iter_mut().zip(x.iter().zip(k)) {
            *o = xi + weight * ki;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// dx/dt = -k x, whose exact solution is known, so integrator error is
    /// measurable rather than eyeballed.
    struct Decay {
        k: f64,
    }

    impl Derivatives for Decay {
        fn derivatives(&mut self, _t: f64, x: &[f64], dx: &mut [f64]) {
            dx[0] = -self.k * x[0];
        }
    }

    /// A harmonic oscillator, which is what a suspension mode looks like to the
    /// integrator, and where an unstable method shows up as growing amplitude.
    struct Oscillator {
        omega: f64,
    }

    impl Derivatives for Oscillator {
        fn derivatives(&mut self, _t: f64, x: &[f64], dx: &mut [f64]) {
            dx[0] = x[1];
            dx[1] = -self.omega * self.omega * x[0];
        }
    }

    #[test]
    fn rk4_is_fourth_order_accurate() {
        let mut system = Decay { k: 3.0 };
        let mut integrator = FixedStep::new(1, Method::Rk4, 1);
        let mut x = [1.0];
        let dt = 1e-3;
        let mut t = 0.0;
        for _ in 0..1000 {
            t = integrator.advance(&mut system, t, &mut x, dt);
        }
        let exact = (-3.0f64).exp();
        assert!(
            (x[0] - exact).abs() < 1e-10,
            "rk4 error {} too large",
            (x[0] - exact).abs()
        );
    }

    #[test]
    fn rk4_holds_a_suspension_mode_at_one_millisecond() {
        // 4 Hz heave mode, the fastest sprung mode in an FSAE car.
        let mut system = Oscillator {
            omega: 2.0 * std::f64::consts::PI * 4.0,
        };
        let mut integrator = FixedStep::new(2, Method::Rk4, 1);
        let mut x = [0.05, 0.0];
        let mut t = 0.0;
        for _ in 0..60_000 {
            t = integrator.advance(&mut system, t, &mut x, 1e-3);
        }
        let energy = 0.5 * x[1] * x[1] + 0.5 * system.omega * system.omega * x[0] * x[0];
        let initial = 0.5 * system.omega * system.omega * 0.05 * 0.05;
        assert!(
            (energy / initial - 1.0).abs() < 1e-3,
            "energy drifted by {:.3}% over 60 s",
            100.0 * (energy / initial - 1.0)
        );
    }

    #[test]
    fn substeps_cost_exactly_what_they_claim() {
        let integrator = FixedStep::new(4, Method::Rk4, 4);
        assert_eq!(integrator.evaluations_per_step(), 16);
    }
}
