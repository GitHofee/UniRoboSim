"""Public entry point for the UniRoboSim teaching Adapter."""

from .adapter import DESCRIPTOR, TeachingProvider, TeachingSession, TeachingWorld, create_provider

__version__ = "0.1.0"

__all__ = [
    "DESCRIPTOR",
    "TeachingProvider",
    "TeachingSession",
    "TeachingWorld",
    "__version__",
    "create_provider",
]
