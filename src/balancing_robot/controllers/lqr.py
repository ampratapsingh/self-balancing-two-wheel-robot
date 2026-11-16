"""LQR controller - implementation pending Phase 1."""
# TODO: implement in Phase 1
from balancing_robot.controllers.base import BaseController


class LQRController(BaseController):
    """Placeholder LQR controller."""

    @property
    def name(self) -> str:
        return "LQR"

    def compute(self, state: dict, dt: float) -> float:
        raise NotImplementedError("LQR controller not yet implemented")

    def reset(self) -> None:
        pass
