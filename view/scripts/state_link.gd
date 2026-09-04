# Read-only attachment to the kernel's published vehicle state.
#
# POSIX shared memory segments appear under /dev/shm on Linux, which means the
# view can read the kernel's state with ordinary file calls and needs no native
# extension at all. It opens the segment READ, so this process physically cannot
# write to the bytes the driver is feeling.
#
# The segment is a seqlock (see kernel/src/transport/seqlock.rs):
#
#   0..8    magic         u64
#   8..16   layout_hash   u64   schema fingerprint
#   16..24  payload_size  u64
#   24..32  sequence      u64   even = stable, odd = write in progress
#   32..    payload
#
# The read protocol is: take the sequence, take the payload, take the sequence
# again, and retry if either it was odd or it changed. That means a frame here
# is always one consistent step of the model, never a mix of two -- and the
# kernel is never blocked by this process reading.
class_name BobDilStateLink
extends RefCounted

const HEADER_SIZE := 32
const OFFSET_MAGIC := 0
const OFFSET_LAYOUT := 8
const OFFSET_PAYLOAD_SIZE := 16
const OFFSET_SEQUENCE := 24

# "BOBDIL01" little-endian, matching transport::seqlock::SEGMENT_MAGIC.
const SEGMENT_MAGIC := 0x31304c494442_4f42

const SHM_DIR := "/dev/shm/"

var _attached := false
var _payload_size := 0
var _segment_path := ""
var last_error := ""
var last_sequence := 0


## Attach to a segment by its schema name. Returns false and sets `last_error`
## rather than throwing, because "the kernel is not running yet" is a normal
## state for the view to be in, not an exception.
func attach(segment_name: String, expected_layout_hash: int, expected_size: int) -> bool:
	_segment_path = SHM_DIR + segment_name
	if not FileAccess.file_exists(_segment_path):
		last_error = "no segment at %s -- is bobdil-kernel running?" % _segment_path
		return false

	var file := FileAccess.open(_segment_path, FileAccess.READ)
	if file == null:
		last_error = "cannot open %s (error %d)" % [_segment_path, FileAccess.get_open_error()]
		return false

	var header := file.get_buffer(HEADER_SIZE)
	file = null
	if header.size() < HEADER_SIZE:
		last_error = "segment is too short to hold a header"
		return false

	var magic := header.decode_u64(OFFSET_MAGIC)
	if magic != SEGMENT_MAGIC:
		last_error = "%s is not a BobDil segment" % _segment_path
		return false

	var layout := header.decode_u64(OFFSET_LAYOUT)
	if layout != expected_layout_hash:
		# Refusing here is the whole point of the layout hash. Reading on would
		# mean drawing the right bytes with the wrong meaning, which looks like
		# a physics bug and is not one.
		last_error = (
			"schema mismatch: the kernel wrote layout %d, this view expects %d. "
			+ "Rebuild both from the same schema (`make codegen`)."
		) % [layout, expected_layout_hash]
		return false

	_payload_size = header.decode_u64(OFFSET_PAYLOAD_SIZE)
	if _payload_size != expected_size:
		last_error = "payload is %d bytes, expected %d" % [_payload_size, expected_size]
		return false

	_attached = true
	last_error = ""
	return true


func is_attached() -> bool:
	return _attached


func detach() -> void:
	_attached = false


## Read the newest stable frame, or an empty PackedByteArray if the writer was
## caught mid-update on every attempt.
##
## The segment is re-opened on every read. That is not laziness: Godot's
## FileAccess buffers, and seeking within a handle held open across frames
## returns the bytes it read the first time -- so a view built on a persistent
## handle renders one frozen frame forever while looking entirely healthy. On
## tmpfs an open/read/close is a few microseconds, which at display rate is
## nothing.
func read_payload(max_attempts: int = 8) -> PackedByteArray:
	if not _attached:
		return PackedByteArray()

	for _attempt in max_attempts:
		var file := FileAccess.open(_segment_path, FileAccess.READ)
		if file == null:
			last_error = "segment disappeared -- the kernel exited"
			_attached = false
			return PackedByteArray()

		var whole := file.get_buffer(HEADER_SIZE + _payload_size)
		file = null
		if whole.size() < HEADER_SIZE + _payload_size:
			return PackedByteArray()

		# The seqlock check still applies. A single read() is not atomic against
		# the writer, so a torn snapshot is possible; an odd sequence, or one
		# that has moved on by the time of the next read, is how it is caught.
		var sequence := whole.decode_u64(OFFSET_SEQUENCE)
		if sequence % 2 != 0:
			continue

		var verify := FileAccess.open(_segment_path, FileAccess.READ)
		if verify == null:
			return PackedByteArray()
		verify.seek(OFFSET_SEQUENCE)
		var after := verify.get_buffer(8)
		verify = null
		if after.size() < 8 or after.decode_u64(0) != sequence:
			continue

		last_sequence = sequence
		return whole.slice(HEADER_SIZE, HEADER_SIZE + _payload_size)

	return PackedByteArray()


## Decode a frame into a Dictionary using the generated offset table.
static func decode(payload: PackedByteArray, offsets: Dictionary, float_fields: Array) -> Dictionary:
	var frame := {}
	if payload.is_empty():
		return frame
	var floats := {}
	for name in float_fields:
		floats[name] = true
	for name in offsets:
		var offset: int = offsets[name]
		if floats.has(name):
			frame[name] = payload.decode_double(offset)
		else:
			frame[name] = payload.decode_u64(offset)
	return frame
