import os


def can_build(env, platform):
    # The module links prebuilt PhysX SDK static libs vendored under
    # thirdparty/physx/lib/<platform>/<arch>/<config> (see SCsub's lib-path
    # table). Only build on platforms whose libs are actually vendored, so
    # CI for platforms without an SDK checkout stays green instead of dying
    # at link time; the module lights up on a platform automatically as soon
    # as its lib directory lands in the tree.
    lib_roots = {
        "windows": "thirdparty/physx/lib/windows/x86_64",
        "macos": "thirdparty/physx/lib/macos/universal",
        "linuxbsd": "thirdparty/physx/lib/linux/x86_64",
    }
    if platform == "android":
        return os.path.isdir("thirdparty/physx/lib/android")
    if platform in lib_roots:
        return os.path.isdir(lib_roots[platform])
    # iOS and web have no SDK wiring yet and would fail at link time.
    return False


def configure(env):
    pass


def get_doc_classes():
    return [
        "PhysXServer3D",
        "PhysXDirectBodyState3D",
        "PhysXChunkEmitter3D",
        "PhysXCloth3D",
        "PhysXParticleFluid3D",
        "PhysXGranular3D",
        "PhysXGas3D",
        "PhysXGasEmitter3D",
        "PhysXBlastAsset",
		"PhysXBlastAuthoring",
		"PhysXDestructible3D",
        "PhysXFlowSimulation3D",
        "PhysXFlowEmitter3D",
        "PhysXFlowCollider3D",
        "PhysXFlowBlastBridge3D",
        "PhysXVehicle3D",
        "PhysXVehicleWheel3D",
        "PhysXMotorcycle3D",
        "PhysXTank3D",
		"PhysXWaterSurface3D",
    ]


def get_doc_path():
    return "doc_classes"
