from abc import ABC, abstractmethod

from aerohacks.core.models import Observation, Plan


class Policy(ABC):
    @abstractmethod
    def step(self, observation: Observation) -> Plan:
        """Return a Plan with exactly 5 ActionSteps."""
        pass
