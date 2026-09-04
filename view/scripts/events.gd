# FSAE event layouts, to competition dimensions.
#
# These are cones, not a road model. BobLib's VehicleFMI runs on
# VehicleInterfaces.Roads.FlatRoad -- a flat infinite plane with no elevation,
# banking or grip variation -- and every FSAE event is run on flat pavement, so
# for Phase 1 the events live entirely in the visual layer (architecture.md 3).
# That is correct for the domain and it keeps a whole road subsystem off the
# critical path. A heightmap-backed Road slots in later behind the same
# VehicleInterfaces.Roads contract without touching anything here.
#
# Positions are in the kernel's frame: +x forward, +y left, metres.
class_name BobDilEvents
extends RefCounted

## FSAE skidpad: two circles of 15.25 m inner diameter, 21.25 m outer,
## centres 18.25 m apart, driven as a figure of eight.
static func skidpad() -> Array[Vector2]:
	var cones: Array[Vector2] = []
	var centre_offset := 18.25 * 0.5
	for side in [-1.0, 1.0]:
		for radius in [15.25 * 0.5, 21.25 * 0.5]:
			var count := int(round(TAU * radius / 3.0))
			for i in count:
				var angle := TAU * float(i) / float(count)
				cones.append(Vector2(
					cos(angle) * radius,
					side * centre_offset + sin(angle) * radius
				))
	return cones


## FSAE acceleration: a 75 m straight, 3 m wide, gated every 5 m.
static func acceleration() -> Array[Vector2]:
	var cones: Array[Vector2] = []
	var gate := 0.0
	while gate <= 75.0:
		cones.append(Vector2(gate, 1.5))
		cones.append(Vector2(gate, -1.5))
		gate += 5.0
	return cones


## A representative autocross course: a closed circuit with a straight, a
## hairpin, a sweeper and a slalom -- the four things a driver forms an opinion
## about. Not a specific competition layout, which changes every year.
static func autocross() -> Array[Vector2]:
	var centreline := _autocross_centreline()
	var cones: Array[Vector2] = []
	var half_width := 2.0
	for i in centreline.size():
		var here: Vector2 = centreline[i]
		var next: Vector2 = centreline[(i + 1) % centreline.size()]
		var heading := (next - here).normalized()
		var left := Vector2(-heading.y, heading.x)
		if i % 2 == 0:
			cones.append(here + left * half_width)
			cones.append(here - left * half_width)
	return cones


static func _autocross_centreline() -> Array[Vector2]:
	var path: Array[Vector2] = []
	# A lemniscate-like loop, stretched into something with a long straight and
	# a tight end, sampled at roughly 2 m intervals.
	var samples := 260
	for i in samples:
		var t := TAU * float(i) / float(samples)
		var x := 55.0 * sin(t)
		var y := 26.0 * sin(t * 2.0) + 8.0 * sin(t * 3.0)
		path.append(Vector2(x, y))
	return path


static func layout(name: String) -> Array[Vector2]:
	match name:
		"skidpad":
			return skidpad()
		"acceleration":
			return acceleration()
		_:
			return autocross()


static func names() -> PackedStringArray:
	return PackedStringArray(["autocross", "skidpad", "acceleration"])
