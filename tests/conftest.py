"""Shared pytest fixtures for the kr6_kinematics test-suite."""

from __future__ import annotations

import numpy as np
import pytest


@pytest.fixture(scope="session")
def rng() -> np.random.Generator:
    """A deterministic RNG shared across the whole test session."""
    return np.random.default_rng(42)
