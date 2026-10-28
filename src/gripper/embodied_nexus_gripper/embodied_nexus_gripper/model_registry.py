"""Lookup for independently registered gripper model providers."""

from importlib.metadata import entry_points


def resolve_model(model: str):
    """Resolve a model provider by identifier.

    Args:
        model: Configured gripper model identifier.

    Returns:
        Model provider.
    """
    if model == "ms42dc":
        from .adapters.ms42dc.model import MS42DCModel
        return MS42DCModel()
    providers = list(entry_points().select(group="embodied_nexus_gripper.models", name=model))
    if len(providers) != 1:
        raise ValueError(f"unsupported or ambiguous gripper model {model!r}")
    return providers[0].load()()
