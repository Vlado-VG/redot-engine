def can_build(env, platform):
    # PhysX ships prebuilt static libs for windows/macos/linux/android only
    # (see SCsub's lib-path table; iOS and web have no SDK wiring yet and
    # would fail at link time).
    return platform in ["windows", "macos", "linuxbsd", "android"]


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
        "PhysXBlastAsset",
    ]


def get_doc_path():
    return "doc_classes"
