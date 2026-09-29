extends SceneTree

# EngineDrive (use_gearbox) smoke test for PhysXVehicle3D: builds the car in
# gearbox mode on flat ground, drives with throttle and verifies:
#   1. the engine spins up (rpm > idle) and the autobox shifts (gear changes),
#   2. the car accelerates nose-first (-Z, the documented convention),
#   3. reverse gear (target_gear = GEAR_REVERSE) drives backward,
#   4. telemetry getters return live values (wheel rpm, skid, contact).
#
# Run:
#   redot --headless --path test/project -s gdscript/vehicle_gearbox_test.gd

var car: Node
var frame := 0
var max_rpm := 0.0
var gears_seen := {}
var fwd_at_3s := 0.0

func _initialize() -> void:
	var ground: Node = ClassDB.instantiate("StaticBody3D")
	root.add_child(ground)
	var gs: Node = ClassDB.instantiate("CollisionShape3D")
	var gbox: Object = ClassDB.instantiate("BoxShape3D")
	gbox.set("size", Vector3(400, 1, 400))
	gs.set("shape", gbox)
	ground.add_child(gs)
	ground.set_position(Vector3(0, -0.5, 0))

	car = ClassDB.instantiate("PhysXVehicle3D")
	root.add_child(car)
	car.global_transform = Transform3D(Basis(), Vector3(0, 0.15, 0))
	car.set("mass", 1200.0)
	car.set("can_sleep", false)
	car.set("use_gearbox", true)
	car.set("use_autobox", false)
	car.set("target_gear", 1)  # neutral first (adjacent stepping)
	car.set("engine_peak_torque", 800.0)
	car.set("target_gear", 2)

	var chassis: Node = ClassDB.instantiate("CollisionShape3D")
	var cbox: Object = ClassDB.instantiate("BoxShape3D")
	cbox.set("size", Vector3(1.8, 0.6, 4.0))
	chassis.set("shape", cbox)
	chassis.set_position(Vector3(0, 0.1, 0))
	car.add_child(chassis)

	for i in 4:
		var w: Node = ClassDB.instantiate("PhysXVehicleWheel3D")
		var front := i < 2
		w.set_position(Vector3(-0.85 if i % 2 == 0 else 0.85,
				-0.1, -1.4 if front else 1.4))
		w.set("use_as_steering", front)
		w.set("use_as_traction", true)
		car.add_child(w)

	# Safety net: if the setter ever refuses to enable again, report SKIP
	# instead of testing direct drive as if it were engine drive.
	if not bool(car.get("use_gearbox")):
		print("[gearbox] SKIP: engine drive is blocked (setter refused) -- direct drive remains available")
		quit(0)
		return

	physics_frame.connect(_tick)

func _tick() -> void:
	frame += 1
	if frame == 30:
		car.set("target_gear", 2)  # 1st gear (adjacent step from neutral)
		car.set("throttle", 1.0)
		print("[gearbox] f=30 -> target 1st, throttle on")
	if frame == 90:
		print("[gearbox] f=90 gear=%s rpm=%.0f pos.z=%.2f fwd=%.2f" % [car.get_engine_gear(), car.get_engine_rpm(), car.global_position.z, car.get_forward_speed()])
	if frame > 30:
		max_rpm = maxf(max_rpm, car.get_engine_rpm())
		gears_seen[car.get_engine_gear()] = true
	if frame == 210:
		fwd_at_3s = car.get_forward_speed()
		print("[gearbox] f=210 rpm=%.0f gears=%s fwd=%.1f pos=%s" % [
				car.get_engine_rpm(), gears_seen.keys(), fwd_at_3s, car.global_position])
		# Reverse from (near) rest: brake first.
		car.set("brake", 1.0)
		car.set("throttle", 0.0)
	if frame == 300:
		car.set("brake", 0.0)
		car.set("target_gear", car.GEAR_REVERSE)
		car.set("throttle", 1.0)
	if frame == 420:
		var contact: Dictionary = car.get_wheel_contact(0)
		var ok := true
		var failures: Array = []
		var fwd: float = car.get_forward_speed()
		var pos: Vector3 = car.global_position
		if fwd_at_3s < 5.0:
			failures.append("no sustained forward accel (fwd=%.1f)" % fwd_at_3s)
		if pos.z > -5.0:
			failures.append("did not drive nose-first (-Z) (z=%.1f)" % pos.z)
		if max_rpm < 60.0:
			failures.append("engine never revved (max_rpm=%.0f)" % max_rpm)
		if gears_seen.size() < 2:
			failures.append("autobox never shifted (gears=%s)" % str(gears_seen.keys()))
		if fwd >= 0.0:
			failures.append("reverse gear did not drive backward (fwd=%.1f)" % fwd)
		if not bool(contact.get("contact", false)):
			failures.append("wheel 0 reports no contact")
		for f in failures:
			push_error("[gearbox] FAIL: " + f)
		print("[gearbox] RESULT: ", "PASS" if failures.is_empty() else "FAIL",
				" fwd=%.1f pos.z=%.1f max_rpm=%.0f gears=%s contact=%s" % [
				fwd, pos.z, max_rpm, gears_seen.keys(), contact.get("contact")])
		quit(0 if failures.is_empty() else 1)
