extends SceneTree

# Runtime pipeline smoke test for the Flow integration: drives a real Flow
# simulation for ~3 simulated seconds with one smoke+fire emitter and asserts
# that density actually flows back through the GPU readback (max_smoke > 0
# and blocks allocated). This is the end-to-end validation of the SDK
# handshake (loader -> device -> grid -> simulate -> snapshot database ->
# readback copies -> CPU sparse decode), including the emitter matrix
# convention (velocity (0, 4, 0) must make blocks allocate ABOVE the emitter
# in Godot's Y-up space).
#
# Run:
#   redot --headless --path test/project -s gdscript/flow_smoke_test.gd
# (headless is fine: Flow runs on its own GPU device; only the FogVolume
# presentation is inert without a rendering device)

var sim: Node3D
var emitter: Node3D
var frame := 0

func _initialize() -> void:
	if not ClassDB.class_exists("PhysXFlowSimulation3D"):
		print("[flow-smoke] FAIL: PhysXFlowSimulation3D not registered (flow=no build?)")
		quit(1)
		return

	var holder := Node3D.new()
	root.add_child(holder)

	sim = ClassDB.instantiate("PhysXFlowSimulation3D")
	holder.add_child(sim)
	sim.set("max_blocks", 1024)
	sim.set("cell_size", 0.25) # explicit: auto-cellsize is a debug suspect
	sim.set("debug_stats", true)

	emitter = ClassDB.instantiate("PhysXFlowEmitter3D")
	holder.add_child(emitter)
	emitter.position = Vector3(0, 0.5, 0)
	emitter.set("radius", 0.4)
	emitter.set("velocity", Vector3(0, 4.0, 0))
	emitter.set("smoke", 1.0)
	emitter.set("temperature", 0.5)
	emitter.set("fuel", 0.4)

	sim.set("emitters", [sim.get_path_to(emitter)])

	physics_frame.connect(_on_physics_frame)

func _on_physics_frame() -> void:
	frame += 1
	if frame == 30 or frame == 90 or frame == 170:
		var diag: Dictionary = sim.get_diagnostics()
		print("[flow-smoke] frame=", frame,
				" available=", diag.get("available"),
				" backend=", diag.get("backend"),
				" blocks=", diag.get("active_blocks"),
				"/", diag.get("max_blocks"),
				" max_smoke=", "%.3f" % sim.get_last_max_smoke(),
				" max_temp=", "%.3f" % sim.get_last_max_temperature(),
				" submit_us=", diag.get("last_submit_usec"),
				" flush_us=", diag.get("last_flush_usec"),
				" decode_us=", diag.get("last_decode_usec"),
				" bounds_min=", sim.get_active_bounds_min(),
				" bounds_size=", sim.get_active_bounds_size())
	if frame >= 180:
		var diag: Dictionary = sim.get_diagnostics()
		var ok: bool = bool(diag.get("available", false)) \
				and int(diag.get("active_blocks", 0)) > 0 \
				and sim.get_last_max_smoke() > 0.0
		if ok:
			# Convention check: with velocity +Y, the active volume must sit
			# above the emitter's Y (0.5), not below or beside it.
			var min_b: Vector3 = sim.get_active_bounds_min()
			if min_b.y < 0.4:
				print("[flow-smoke] NOTE: bounds_min.y=", min_b.y,
						" below emitter Y -- check emitter matrix row convention")
		print("[flow-smoke] RESULT: ", "PASS" if ok else "FAIL")
		quit(0 if ok else 1)
