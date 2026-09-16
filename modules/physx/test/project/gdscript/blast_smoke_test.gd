extends SceneTree

# Headless smoke test for the Blast (destructible mesh) extension:
#   authoring (fracture) -> PhysXBlastAsset -> PhysXDestructible3D lifecycle
#   (load, spawn intact, radial damage -> split into rigid pieces).
# Run from the repository root:
#   bin/redot.windows.editor.x86_64.mono.console.exe --headless --path modules/physx/test/project --script res://gdscript/blast_smoke_test.gd
# Exit code 0 = pass, 1 = fail.

var _failed := false

func _fail(msg: String) -> void:
	print("BLAST-TEST FAIL: ", msg)
	_failed = true

func _init() -> void:
	var authoring := PhysXBlastAuthoring.new()
	if authoring == null:
		_fail("PhysXBlastAuthoring not registered")
		quit(1)
		return

	var mesh := BoxMesh.new()
	mesh.size = Vector3(2, 2, 2)

	# Voronoi: the default pattern, deterministic for a fixed seed.
	var asset := authoring.fracture_mesh(mesh, 12, 12345)
	if asset == null or asset.get_chunk_count() < 2:
		_fail("voronoi fracture produced no chunks")
	else:
		print("BLAST-TEST voronoi chunks: ", asset.get_chunk_count())

	# Slicing: cube-root split across the axes.
	var slicing_asset := authoring.fracture_mesh(mesh, 8, 7, PhysXBlastAuthoring.FracturePattern.PATTERN_SLICING)
	if slicing_asset == null or slicing_asset.get_chunk_count() < 2:
		_fail("slicing fracture produced no chunks")
	else:
		print("BLAST-TEST slicing chunks: ", slicing_asset.get_chunk_count())

	# Round-trip through Resource serialization (the .tres save path the
	# editor fracture dialog uses).
	if asset != null:
		asset.resource_path = "res://blast_smoke_asset.tres"
		var err := ResourceSaver.save(asset, asset.resource_path)
		if err != OK:
			_fail("ResourceSaver.save failed with error %d" % err)
		else:
			var loaded := ResourceLoader.load("res://blast_smoke_asset.tres") as PhysXBlastAsset
			if loaded == null or loaded.get_chunk_count() != asset.get_chunk_count():
				_fail("asset save/load round-trip lost chunks")
			else:
				print("BLAST-TEST resource round-trip ok (", loaded.get_chunk_count(), " chunks)")
			DirAccess.remove_absolute(ProjectSettings.globalize_path("res://blast_smoke_asset.tres"))

	# Destructible node: load, spawn the intact body, fracture it, count pieces.
	if asset != null:
		var destructible := PhysXDestructible3D.new()
		destructible.blast_asset = asset
		destructible.dynamic = false
		root.add_child(destructible)
		# Give ENTER_WORLD a chance to load the asset and spawn the intact
		# body before damaging it.
		await process_frame
		await physics_frame

		var pieces_before := destructible.apply_radial_damage(Vector3(0, 2, 0), 10.0, 0.0, 5.0)
		if pieces_before < 1:
			_fail("apply_radial_damage spawned no pieces")
		else:
			print("BLAST-TEST radial damage spawned pieces: ", pieces_before)

		# Let a few physics ticks run so the spawned rigid bodies simulate
		# (sync transform pass + kill-floor logic must not crash headless).
		await physics_frame
		await physics_frame
		await physics_frame

		destructible.free()

	if _failed:
		quit(1)
	else:
		print("BLAST-TEST PASS")
		quit(0)
