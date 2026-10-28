"""Lookup for independently registered lidar model providers."""

from importlib.metadata import entry_points


def resolve_model(model: str):
    """Resolve a lidar provider by identifier.

    Args:
        model: Configured lidar model ID.

    Returns:
        Model provider.
    """
    if model == "lslidar_c16":
        from .adapters.c16.model import C16Model
        return C16Model()
    providers = list(entry_points().select(group="embodied_nexus_lidar.models", name=model))
    if len(providers) != 1:
        raise ValueError(f"unsupported or ambiguous lidar model {model!r}")
    return providers[0].load()()
