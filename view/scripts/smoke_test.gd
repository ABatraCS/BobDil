# Headless check that the view can actually read what the kernel publishes.
#
# Run with the kernel already running:
#     bobdil-kernel run --duration 10 &
#     godot --headless --path view --script scripts/smoke_test.gd
#
# This is the integration test for the one interface between the two processes.
# It fails loudly on a schema mismatch, which is the failure that would
# otherwise show up as physics that looks subtly wrong.
extends SceneTree

const SAMPLES := 120


func _initialize() -> void:
	var link := BobDilStateLink.new()
	if not link.attach(
		BobDilFrames.VEHICLE_STATE_SHM,
		BobDilFrames.LAYOUT_HASH,
		BobDilFrames.VEHICLE_STATE_SIZE
	):
		printerr("FAIL attach: ", link.last_error)
		quit(1)
		return

	print("attached to /dev/shm/", BobDilFrames.VEHICLE_STATE_SHM,
		"  layout ", BobDilFrames.LAYOUT_HASH,
		"  payload ", BobDilFrames.VEHICLE_STATE_SIZE, " B")

	var torn := 0
	var advanced := 0
	var previous_step := -1
	var fastest := 0.0
	var last_frame := {}

	for i in SAMPLES:
		var payload := link.read_payload()
		if payload.is_empty():
			torn += 1
			continue
		var frame := BobDilStateLink.decode(
			payload,
			BobDilFrames.VEHICLE_STATE_OFFSETS,
			BobDilFrames.VEHICLE_STATE_FLOAT_FIELDS
		)
		last_frame = frame
		var step := int(frame["step_index"])
		if step > previous_step:
			advanced += 1
		previous_step = step
		fastest = maxf(fastest, frame["vehicle_speed"])
		OS.delay_msec(8)

	print("reads: %d attempted, %d gave up mid-write, %d showed a newer step" % [SAMPLES, torn, advanced])
	print("newest frame: t=%.3f s  step=%d  speed=%.2f m/s  torque=%+.2f N.m  rtf=%.3f  kernel=%d" % [
		last_frame.get("sim_time", 0.0),
		int(last_frame.get("step_index", 0)),
		last_frame.get("vehicle_speed", 0.0),
		last_frame.get("ffb_torque_nm", 0.0),
		last_frame.get("rtf", 0.0),
		int(last_frame.get("kernel_id", 0)),
	])

	if advanced < SAMPLES / 4:
		printerr("FAIL: the state segment is not advancing; is the kernel running?")
		quit(1)
		return
	if fastest <= 0.5:
		printerr("FAIL: the car never moved, so the view is not reading real physics")
		quit(1)
		return
	print("PASS: the view reads live, consistent, advancing physics from the kernel.")
	quit(0)
