import numpy as np


class PurePursuitLaneController:
    """
    The Lane Controller can be used to compute control commands from pose estimations.

    The control commands are in terms of linear and angular velocity (v, omega). The input are errors in the relative
    pose of the Duckiebot in the current lane.

    """

    def __init__(self, parameters):

        self.parameters = parameters

    def update_parameters(self, parameters):
        """Updates parameters of LaneController object.

            Args:
                parameters (:obj:`dict`): dictionary containing the new parameters for LaneController object.
        """
        self.parameters = parameters


    def pure_pursuit(self):
        
        hypothenuse = np.sqrt(self.parameters["target"].dot(self.parameters["target"]))
        sin_alpha = self.parameters["target"][1] / hypothenuse
        min_speed = 0.1
        max_speed = 1.0
        v = self.parameters["max_velocity"] * (1 - abs(sin_alpha))
        v = np.clip(v, min_speed, max_speed)
        omega = 2 * sin_alpha / self.parameters["lookahead_dist"]
        
        return v,omega
