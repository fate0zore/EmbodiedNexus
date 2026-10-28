"""Lookup for independently registered arm model adapters."""

from importlib.metadata import entry_points


def resolve_model(model: str):
    """Resolve an installed model provider by its declared identifier.

    Args:
        model: Model identifier from the device configuration.

    Returns:
        Model provider implementing validation, backend and operations creation.
    """
    if model == "aubo_i5":
        from .adapters.aubo_i5.model import AuboI5Model
        return AuboI5Model()
    providers = list(entry_points().select(group="embodied_nexus_arm.models", name=model))
    if len(providers) != 1:
        raise ValueError(f"unsupported or ambiguous arm model {model!r}")
    return providers[0].load()()
