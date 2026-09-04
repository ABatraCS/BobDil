# The driver's point of view.
#
# This scene is built entirely in code so there is one place to read, and so
# nothing about the world can be edited into disagreement with the kernel.
#
# The view's whole contract with physics is `BobDilStateLink`: attach to a
# read-only shared-memory segment, take the newest frame, draw it. It never
# writes, never blocks the kernel, and if it stalls for a frame the plant does
# not notice -- a dropped frame here is correct behaviour, which is exactly why
# the transport is a seqlock and not a queue.
extends Node3D

const CONE_HEIGHT := 0.32
const CONE_RADIUS := 0.14
const EYE_HEIGHT := 0.62
const EYE_FORWARD := 0.25

var _link := BobDilStateLink.new()
var _hud: BobDilHud
var _car: Node3D
var _camera: Camera3D
var _cones: MultiMeshInstance3D

var _event_index := 0
var _cockpit_view := true
var _retry_countdown := 0.0
var _last_frame := {}
## Smoothed pose. The kernel publishes at 1 kHz and the view draws at display
## rate, so the newest frame is up to one display period old; interpolating
## toward it removes the visible stepping without ever inventing motion the
## model did not produce.
var _drawn_position := Vector3.ZERO
var _drawn_yaw := 0.0


func _ready() -> void:
	_build_world()
	_build_car()
	_build_camera()
	_hud = BobDilHud.new()
	add_child(_hud)
	_load_event(0)
	_try_attach()


func _try_attach() -> void:
	if _link.attach(
		BobDilFrames.VEHICLE_STATE_SHM,
		BobDilFrames.LAYOUT_HASH,
		BobDilFrames.VEHICLE_STATE_SIZE
	):
		_hud.set_capabilities("")
	else:
		_hud.show_disconnected(_link.last_error)
		_retry_countdown = 1.0


func _process(delta: float) -> void:
	if not _link.is_attached():
		_retry_countdown -= delta
		if _retry_countdown <= 0.0:
			_try_attach()
		return

	var payload := _link.read_payload()
	if not payload.is_empty():
		_last_frame = BobDilStateLink.decode(
			payload,
			BobDilFrames.VEHICLE_STATE_OFFSETS,
			BobDilFrames.VEHICLE_STATE_FLOAT_FIELDS
		)

	if _last_frame.is_empty():
		return

	_apply_pose(delta)
	_hud.update(_last_frame, BobDilEvents.names()[_event_index], _kernel_name())


## Kernel frame is +x forward, +y left, z up. Godot is +x right, +y up,
## -z forward. One conversion, in one place.
static func to_godot(x: float, y: float) -> Vector3:
	return Vector3(-y, 0.0, -x)


func _apply_pose(delta: float) -> void:
	var target := to_godot(_last_frame.get("pos_x", 0.0), _last_frame.get("pos_y", 0.0))
	var target_yaw: float = _last_frame.get("yaw", 0.0)

	# Snap rather than smooth when the car has been reset, so a session restart
	# does not send the camera flying across the map.
	var blend := 1.0 - exp(-24.0 * delta)
	if _drawn_position.distance_to(target) > 25.0:
		blend = 1.0
	_drawn_position = _drawn_position.lerp(target, blend)
	_drawn_yaw = lerp_angle(_drawn_yaw, target_yaw, blend)

	_car.position = _drawn_position
	_car.rotation = Vector3(
		0.0,
		_drawn_yaw,
		# Roll is drawn because it is one of the few visual cues that stands in
		# for the motion cue a static rig cannot deliver.
		-_last_frame.get("roll", 0.0)
	)

	if _cockpit_view:
		_camera.position = _car.position + _car.transform.basis * Vector3(0.0, EYE_HEIGHT, -EYE_FORWARD)
		_camera.rotation = Vector3(0.0, _drawn_yaw, -_last_frame.get("roll", 0.0) * 0.35)
	else:
		var behind := _car.transform.basis * Vector3(0.0, 2.4, 6.5)
		_camera.position = _car.position + behind
		_camera.look_at(_car.position + Vector3(0.0, 0.6, 0.0), Vector3.UP)


func _kernel_name() -> String:
	match int(_last_frame.get("kernel_id", 0)):
		BobDilFrames.KERNEL_ID_VEHICLE_FMI:
			return "VehicleFMI (full Modelica)"
		BobDilFrames.KERNEL_ID_VEHICLE_RT:
			return "VehicleRT (realtime Modelica)"
		BobDilFrames.KERNEL_ID_REDUCED14DOF:
			return "Reduced14Dof (fallback plant)"
		_:
			return "unknown plant"


func _unhandled_input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_C:
			_cockpit_view = not _cockpit_view
		KEY_TAB:
			_load_event((_event_index + 1) % BobDilEvents.names().size())
		KEY_ESCAPE:
			get_tree().quit()


# --- world construction -------------------------------------------------

func _build_world() -> void:
	var environment := WorldEnvironment.new()
	var settings := Environment.new()
	settings.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var material := ProceduralSkyMaterial.new()
	material.sky_top_color = Color(0.31, 0.45, 0.60)
	material.sky_horizon_color = Color(0.71, 0.75, 0.78)
	material.ground_bottom_color = Color(0.22, 0.23, 0.24)
	material.ground_horizon_color = Color(0.55, 0.56, 0.57)
	sky.sky_material = material
	settings.sky = sky
	settings.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	settings.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	environment.environment = settings
	add_child(environment)

	var sun := DirectionalLight3D.new()
	sun.rotation = Vector3(deg_to_rad(-52.0), deg_to_rad(35.0), 0.0)
	sun.light_energy = 1.15
	sun.shadow_enabled = true
	add_child(sun)

	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(600.0, 600.0)
	ground.mesh = plane
	var asphalt := StandardMaterial3D.new()
	asphalt.albedo_color = Color(0.235, 0.243, 0.255)
	asphalt.roughness = 0.95
	ground.material_override = asphalt
	add_child(ground)


func _build_car() -> void:
	_car = Node3D.new()
	add_child(_car)

	var body := MeshInstance3D.new()
	var chassis := BoxMesh.new()
	# Roughly an FSAE car: 2.9 m long, 1.4 m wide, 0.5 m of visible bodywork.
	chassis.size = Vector3(1.40, 0.50, 2.90)
	body.mesh = chassis
	body.position = Vector3(0.0, 0.42, 0.0)
	var livery := StandardMaterial3D.new()
	livery.albedo_color = Color(0.05, 0.43, 0.41)
	livery.metallic = 0.25
	livery.roughness = 0.35
	body.material_override = livery
	_car.add_child(body)

	var wheel_mesh := CylinderMesh.new()
	wheel_mesh.top_radius = 0.2286
	wheel_mesh.bottom_radius = 0.2286
	wheel_mesh.height = 0.20
	var rubber := StandardMaterial3D.new()
	rubber.albedo_color = Color(0.09, 0.09, 0.10)
	rubber.roughness = 0.9
	for corner in [
		Vector3(0.61, 0.2286, -0.78),
		Vector3(-0.61, 0.2286, -0.78),
		Vector3(0.59, 0.2286, 0.77),
		Vector3(-0.59, 0.2286, 0.77),
	]:
		var wheel := MeshInstance3D.new()
		wheel.mesh = wheel_mesh
		wheel.material_override = rubber
		wheel.position = corner
		wheel.rotation = Vector3(0.0, 0.0, deg_to_rad(90.0))
		_car.add_child(wheel)


func _build_camera() -> void:
	_camera = Camera3D.new()
	_camera.fov = 78.0
	_camera.near = 0.05
	_camera.far = 900.0
	_camera.current = true
	add_child(_camera)


func _load_event(index: int) -> void:
	_event_index = index
	var layout := BobDilEvents.layout(BobDilEvents.names()[index])

	if _cones != null:
		_cones.queue_free()
	_cones = MultiMeshInstance3D.new()
	var multimesh := MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D

	var cone := CylinderMesh.new()
	cone.top_radius = 0.01
	cone.bottom_radius = CONE_RADIUS
	cone.height = CONE_HEIGHT
	multimesh.mesh = cone
	multimesh.instance_count = layout.size()
	for i in layout.size():
		var here: Vector2 = layout[i]
		multimesh.set_instance_transform(
			i,
			Transform3D(Basis.IDENTITY, to_godot(here.x, here.y) + Vector3(0.0, CONE_HEIGHT * 0.5, 0.0))
		)

	var plastic := StandardMaterial3D.new()
	plastic.albedo_color = Color(0.95, 0.42, 0.10)
	plastic.roughness = 0.6
	_cones.multimesh = multimesh
	_cones.material_override = plastic
	add_child(_cones)
