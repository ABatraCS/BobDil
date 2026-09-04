# The driver's instrument overlay.
#
# Two of these readouts are not conveniences.
#
# The realtime factor is a first-class element, not a diagnostic
# (architecture.md 1.8). If the loop stops keeping up, the driver has to be able
# to see it -- otherwise their verdict on a setup silently encodes the stutter
# instead of the car.
#
# The capability banner is there because the spec asks for pedal stiffness
# feedback that no ordinary pedal set can deliver (architecture.md 5.3). A cue
# that is missing must be labelled as missing, or the driver attributes its
# absence to the vehicle.
class_name BobDilHud
extends CanvasLayer

const OK_COLOUR := Color(0.62, 0.85, 0.72)
const WARN_COLOUR := Color(0.95, 0.78, 0.35)
const CRITICAL_COLOUR := Color(0.94, 0.45, 0.42)
const DIM_COLOUR := Color(0.78, 0.82, 0.85)

var _speed: Label
var _detail: Label
var _health: Label
var _banner: Label
var _caveat: Label


func _ready() -> void:
	layer = 10

	var margin := MarginContainer.new()
	margin.set_anchors_preset(Control.PRESET_FULL_RECT)
	margin.add_theme_constant_override("margin_left", 28)
	margin.add_theme_constant_override("margin_top", 22)
	margin.add_theme_constant_override("margin_right", 28)
	margin.add_theme_constant_override("margin_bottom", 22)
	add_child(margin)

	var rows := VBoxContainer.new()
	rows.add_theme_constant_override("separation", 6)
	margin.add_child(rows)

	_speed = _make_label(rows, 46, Color.WHITE)
	_detail = _make_label(rows, 16, DIM_COLOUR)
	_health = _make_label(rows, 16, OK_COLOUR)

	var spacer := Control.new()
	spacer.size_flags_vertical = Control.SIZE_EXPAND_FILL
	rows.add_child(spacer)

	_banner = _make_label(rows, 16, WARN_COLOUR)
	_caveat = _make_label(rows, 13, DIM_COLOUR)
	_caveat.text = (
		"Static rig: no motion cue. Valid for comparing setups against each other; "
		+ "not valid for judging absolute grip."
	)


func _make_label(parent: Node, size: int, colour: Color) -> Label:
	var label := Label.new()
	label.add_theme_font_size_override("font_size", size)
	label.add_theme_color_override("font_color", colour)
	label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.75))
	label.add_theme_constant_override("outline_size", 6)
	parent.add_child(label)
	return label


func set_capabilities(text: String) -> void:
	_banner.text = text
	_banner.add_theme_color_override(
		"font_color", WARN_COLOUR if text != "" else OK_COLOUR
	)


func show_disconnected(reason: String) -> void:
	_speed.text = "--"
	_detail.text = "waiting for the kernel"
	_health.text = reason
	_health.add_theme_color_override("font_color", WARN_COLOUR)


func update(frame: Dictionary, event_name: String, kernel_name: String) -> void:
	var speed_kph: float = frame.get("vehicle_speed", 0.0) * 3.6
	_speed.text = "%3.0f km/h" % speed_kph

	_detail.text = "%s   |   %s   |   %+.2f g lat   %+.2f g long   |   steer %+.0f deg   torque %+.1f N.m" % [
		event_name,
		kernel_name,
		frame.get("acc_y", 0.0) / 9.80665,
		frame.get("acc_x", 0.0) / 9.80665,
		rad_to_deg(frame.get("handwheel_angle", 0.0)),
		frame.get("ffb_torque_nm", 0.0),
	]

	var rtf: float = frame.get("rtf", 0.0)
	var faults: int = int(frame.get("fault_flags", 0))
	var misses: int = int(frame.get("deadline_misses", 0))
	_health.text = "realtime %.3f x   |   step p99 %.0f us   |   %d missed deadlines%s" % [
		rtf,
		frame.get("step_time_p99_us", 0.0),
		misses,
		"   |   FAULTS: " + _fault_names(faults) if faults != 0 else "",
	]

	# The realtime factor is the one number that invalidates everything else, so
	# it drives the colour rather than being a line of text among many.
	var colour := OK_COLOUR
	if rtf < 0.97 or faults != 0:
		colour = WARN_COLOUR
	if rtf < 0.90:
		colour = CRITICAL_COLOUR
	_health.add_theme_color_override("font_color", colour)


func _fault_names(flags: int) -> String:
	var names := PackedStringArray()
	if flags & BobDilFrames.FAULT_FLAGS_PLANT_NONFINITE:
		names.append("plant diverged")
	if flags & BobDilFrames.FAULT_FLAGS_DEADLINE_MISSED:
		names.append("deadlines missed")
	if flags & BobDilFrames.FAULT_FLAGS_FFB_CLAMPED:
		names.append("torque clamped (you are feeling the limiter, not the car)")
	if flags & BobDilFrames.FAULT_FLAGS_TELEMETRY_OVERFLOW:
		names.append("telemetry dropped -- this run cannot be used for A/B")
	if flags & BobDilFrames.FAULT_FLAGS_PLANT_STEP_FAILED:
		names.append("plant step failed")
	if flags & BobDilFrames.FAULT_FLAGS_INPUT_STALE:
		names.append("input stale")
	return ", ".join(names)
