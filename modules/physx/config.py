def can_build(env, platform):
    return True


def configure(env):
    pass


def get_doc_classes():
    return [
        "PhysXServer3D",
        "PhysXDirectBodyState3D",
        "PhysXChunkEmitter3D",
        "PhysXCloth3D",
        "PhysXParticleFluid3D",
    ]


def get_doc_path():
    return "doc_classes"
