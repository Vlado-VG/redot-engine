extends SceneTree

# End-to-end smoke test for the node-level vehicle stack: builds a
# PhysXVehicle3D (4x PhysXVehicleWheel3D + BoxShape3D chassis) on a ground
# plane, drives it with throttle for ~2 simulated seconds, and asserts the
# car actually moves forward with suspension loaded. Exercises: node build
# (scene borrow via PhysXServer3D::get_space), PxVehicle2 component sequence,
# wheel node transform sync, forward-speed/jounce telemetry.
#
# Run:
#   redot --headless --path test/project -s gdscript/vehicle_node_test.gd

var car: Node3D
var wheels: Array = []
var frame := 0

func _initialize() -> void:
	for cls in ["PhysXVehicle3D", "PhysXVehicleWheel3D", "PhysXMotorcycle3D", "PhysXTank3D"]:
		if not ClassDB.class_exists(cls):
			print("[veh-node] FAIL: ", cls, " not registered")
			quit(1)
			return

	# Ground: big static box (a plane works too; a box keeps the raycast
	# normal trivially up everywhere the car can reach in 2 s).
	var ground: Node = ClassDB.instantiate("StaticBody3D")
	root.add_child(ground)
	var gs: Node = ClassDB.instantiate("CollisionShape3D")
	var gbox: Object = ClassDB.instantiate("BoxShape3D")
	gbox.set("size", Vector3(200, 1, 200))
	gs.set("shape", gbox)
	ground.add_child(gs)
	ground.set_position(Vector3(0, -0.5, 0))

	# Car: PhysXVehicle3D + chassis box + 4 wheels (front z = -1.4 steers).
	car = ClassDB.instantiate("PhysXVehicle3D")
	root.add_child(car)
	car.set_position(Vector3(0, 1.0, 0))
	car.set("mass", 1200.0)
	car.set("can_sleep", false)

	var chassis: Node = ClassDB.instantiate("CollisionShape3D")
	var cbox: Object = ClassDB.instantiate("BoxShape3D")
	cbox.set("size", Vector3(1.8, 0.6, 4.0))
	chassis.set("shape", cbox)
	chassis.set_position(Vector3(0, 0.1, 0))
	car.add_child(chassis)

	for i in 4:
		var w: Node = ClassDB.instantiate("PhysXVehicleWheel3D")
		# Godot convention: steered (front) axle on the -Z side.
		var front := i < 2
		w.set_position(Vector3(-0.85 if i % 2 == 0 else 0.85,
				-0.1, -1.4 if front else 1.4))
		w.set("use_as_steering", front)
		w.set("use_as_traction", true)
		car.add_child(w)
		wheels.append(w)

	car.set("throttle", 1.0)
	physics_frame.connect(_on_physics_frame)

func _on_physics_frame() -> void:
	frame += 1
	if frame == 30 or frame == 120:
		var fs: float = car.get_forward_speed()
		var jounce: float = car.get_wheel_jounce(0)
		var pos: Vector3 = car.get_position()
		print("[veh-node] frame=", frame, " fwd_speed=", "%.2f" % fs,
				" jounce0=", "%.3f" % jounce, " pos=", pos,
				" sleeping=", car.is_sleeping())
	if frame >= 120:
		var fs: float = car.get_forward_speed()
		var jounce: float = car.get_wheel_jounce(0)
		var pos: Vector3 = car.get_position()
		# Forward = -Z: driving with throttle must produce negative z travel
		# and meaningful speed; suspension must be compressed off full travel.
		var moved: bool = pos.z < -1.0
		var speed_ok: bool = fs > 1.0
		var suspension_ok := jounce > 0.0
		print("[veh-node] RESULT: ", "PASS" if (moved and speed_ok and suspension_ok) else "FAIL",
				" (moved=", moved, " speed_ok=", speed_ok, " suspension_ok=", suspension_ok, ")")
		quit(0 if (moved and speed_ok and suspension_ok) else 1)
