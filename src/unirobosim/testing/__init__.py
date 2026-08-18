"""Explicit test utilities; never an implicit production backend."""

from .fake_backend import FAKE_CAPABILITIES, FAKE_DESCRIPTOR, FakeProvider, FakeSession, FakeWorld

__all__ = ["FAKE_CAPABILITIES", "FAKE_DESCRIPTOR", "FakeProvider", "FakeSession", "FakeWorld"]
