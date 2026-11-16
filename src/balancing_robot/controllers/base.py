"""Abstract base class for all controllers."""
from __future__ import annotations
from abc import ABC, abstractmethod


class BaseController(ABC):
    """Common interface that all controllers must implement."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable controller name."""
        ...

    @abstractmethod
    def compute(self, state: dict, dt: float) -> float:
        """Compute total torque command for this timestep.

        Args:
            state: dict with keys: x (m), x_dot (m/s), theta (rad), theta_dot (rad/s)
            dt: timestep in seconds

        Returns:
            Total torque (Nm). Split equally to left/right wheels by the caller.
            Positive torque → forward motion (+X).
        """
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset controller internal state (integrators, filters)."""
        ...
