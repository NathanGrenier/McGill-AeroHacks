from dataclasses import dataclass
from enum import Enum
from typing import List, Optional


# Dummy base class for Region (since the simulator uses PolygonRegion/CircleRegion internally)
class Region:
    pass


@dataclass
class Position2D:
    x: float
    y: float


@dataclass
class State:
    position: Position2D
    alt_layer: int  # Discrete altitude layer (0 = ground)
    energy: float  # Remaining battery energy
    velocity: Optional[Position2D] = None  # Current velocity vector
    heading: Optional[float] = None  # Current heading in radians


class ActionType(Enum):
    WAYPOINT = "WAYPOINT"  # Move to target position/altitude
    HOLD = "HOLD"  # Hover in place
    EMERGENCY_LAND = "EMERGENCY_LAND"  # Descend to ground at current position


@dataclass
class ActionStep:
    action_type: ActionType
    target_position: Optional[Position2D] = None  # Required for WAYPOINT
    target_alt_layer: Optional[int] = None  # Optional altitude change


@dataclass
class Plan:
    steps: List[ActionStep]  # Must contain EXACTLY 5 ActionStep items


class ConstraintPhase(Enum):
    ADVISORY = "ADVISORY"  # Warning only, minor penalty
    CONTROLLED = "CONTROLLED"  # Active penalty zone
    RESTRICTED = "RESTRICTED"  # Heavy penalty, 5+ consecutive steps = catastrophic


@dataclass
class Constraint:
    id: str
    region: Region  # PolygonRegion or CircleRegion
    alt_layers: List[int]  # Affected altitude layers
    phase: ConstraintPhase
    start_time: int
    end_time: Optional[int] = None


@dataclass
class TrafficTrack:
    id: str
    position: Position2D  # Current position
    alt_layer: int  # Current altitude layer
    velocity: Optional[Position2D] = None  # Velocity vector
    intent: Optional[List[Position2D]] = None  # Future positions (predicted)


@dataclass
class GoalRegion:
    region: Region  # Target area (polygon or circle)
    target_alt_layer: Optional[int] = None  # Required altitude to complete mission


@dataclass
class Observation:
    current_time: int  # Current simulation tick
    ownship_state: State  # Your drone's current state
    mission_goal: GoalRegion  # Where you need to go
    active_constraints: List[Constraint]  # Currently active airspace constraints
    traffic_tracks: List[TrafficTrack]  # Other drones in the airspace
