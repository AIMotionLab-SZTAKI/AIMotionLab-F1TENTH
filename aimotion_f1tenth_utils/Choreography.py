from aimotion_f1tenth_utils.Trajectory import Trajectory, ScheduledTrajectory
from typing import List


class Choreography:
    def __init__(self, ID = "", scheduled_trajectories: List[ScheduledTrajectory] = []) -> None:
        """Class implementation of a choreography (i.e. multiple trajectories for multiple agents) for autonomous ground vehicles"""
        self.ID=ID
        self.scheduled_trajectories = scheduled_trajectories 
            
    def add_scheduled_trajectory(self, scheduled_trajectory: ScheduledTrajectory) -> None:
        """Adds a scheduled trajectory to the choreography"""
        self.scheduled_trajectories.append(scheduled_trajectory)

    def schedule_trajectories_by_delay(self, trajectories: List[Trajectory], delays: List[float]):
        self.scheduled_trajectories = [ScheduledTrajectory(trajectory, delay) for trajectory, delay in zip(trajectories, delays)]
        pass

    def schedule_trajectories_by_time(self, trajectories: List[Trajectory], times: List[float]):
        pass

