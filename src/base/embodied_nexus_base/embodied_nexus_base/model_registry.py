"""Lookup for independently registered base model providers."""

from importlib.metadata import entry_points


def resolve_model(model: str):
    """Resolve a base model provider by identifier.

    Args:
        model: Model identifier from the configuration.

    Returns:
        Model provider.
    """
    if model == "scout_2":
        from .adapters.scout_2.model import Scout2Model
        return Scout2Model()
    providers = list(entry_points().select(group="embodied_nexus_base.models", name=model))
    if len(providers) != 1:
        raise ValueError(f"unsupported or ambiguous base model {model!r}")
    return providers[0].load()()
