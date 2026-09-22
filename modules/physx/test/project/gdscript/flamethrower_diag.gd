extends SceneTree

# Diagnostic for the godot-physx-example flamethrower_liquid freeze: rebuilds
# the demo's exact Muzzle fluid config (MPM, 15000 particles at 0.02 m in a
# 5x20x12 m domain, 4 substeps, isosurface on, auto colliders) and measures
# per-tick wall time so a pathological step shows up immediately.

var fluid: PhysXParticleFluid3D
var ground: StaticBody3D
var target: StaticBody3D
var frames := 0
var last_us := 0
var worst_us := 0

func _initialize() -> void:
	print("[diag] engine = ", ProjectSettings.get_setting("physics/3d/physics_engine", "?"))

	ground = StaticBody3D.new()
	var gcs := CollisionShape3D.new()
	var gs := BoxShape3D.new()
	gs.size = Vector3(20, 0.1, 20)
	gcs.shape = gs
	ground.add_child(gcs)
	root.add_child(ground)

	target = StaticBody3D.new()
	var tcs := CollisionShape3D.new()
	var ts := BoxShape3D.new()
	ts.size = Vector3(3, 3, 0.3)
	tcs.shape = ts
	target.add_child(tcs)
	target.position = Vector3(0, 1.5, -4)
	root.add_child(target)

	fluid = PhysXParticleFluid3D.new()
	fluid.solver = PhysXParticleFluid3D.SOLVER_MPM
	fluid.particle_count = 15000
	fluid.particle_size = 0.02
	fluid.mpm_domain_size = Vector3(5, 20, 12)
	fluid.mpm_substeps = 4
	fluid.mpm_auto_colliders = true
	fluid.spawn_region_size = Vector3(0.3, 0.3, 0.3)
	fluid.spawn_on_ready = false
	fluid.emission_rate = 1500.0
	fluid.emission_radius = 0.172
	fluid.emission_velocity = Vector3(0, 0, -9)
	fluid.surface_mesh = true
	fluid.viscosity = 0.207
	fluid.surface_tension = 0.097
	fluid.cohesion = 0.1
	root.add_child(fluid)
	# Collider paths only resolve once the nodes are inside the tree; wire them
	# on the first tick instead of here (see mpm_foam_test for the same pattern).
	fluid.emitting = true # the rig fires on LMB; drive it directly
	last_us = Time.get_ticks_usec()

func _physics_process(_delta: float) -> bool:
	frames += 1
	if frames == 1:
		fluid.mpm_colliders = [ground.get_path(), target.get_path()]
	var now := Time.get_ticks_usec()
	var tick_us := now - last_us
	last_us = now
	worst_us = maxi(worst_us, tick_us)
	if frames <= 5 or frames % 15 == 0:
		print("[diag] frame %3d  tick=%8.1f ms  live=%d" % [
				frames, tick_us / 1000.0, fluid.get_live_particle_count()])
	if frames >= 90:
		print("[diag] worst tick = %.1f ms over %d frames" % [worst_us / 1000.0, frames])
		quit(0)
		return true
	return false
