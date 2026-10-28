"""Lookup for independently registered camera model providers."""

from importlib.metadata import entry_points


def resolve_model(model: str):
    """Resolve a camera model provider.

    Args:
        model: Configured camera model identifier.

    Returns:
        Model provider.
    """
    if model == "gemini335":
        from .adapters.gemini335.model import Gemini335Model
        return Gemini335Model()
    providers = list(entry_points().select(group="embodied_nexus_camera.models", name=model))
    if len(providers) != 1:
        raise ValueError(f"unsupported or ambiguous camera model {model!r}")
    return providers[0].load()()
