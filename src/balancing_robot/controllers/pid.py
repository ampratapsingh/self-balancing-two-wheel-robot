"""PID controller - implementation pending Phase 1."""
# TODO: implement in Phase 1
from balancing_robot.controllers.base import BaseController


class PIDController(BaseController):
    """Placeholder PID controller."""

    @property
    def name(self) -> str:
        return "PID"

    def compute(self, state: dict, dt: float) -> float:
        raise NotImplementedError("PID controller not yet implemented")

    def reset(self) -> None:
        pass
