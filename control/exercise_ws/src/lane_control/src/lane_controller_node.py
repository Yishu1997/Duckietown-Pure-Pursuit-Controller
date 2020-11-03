#!/usr/bin/env python3
import collections
import numpy as np
import rospy
import enum
import itertools
from threading import Lock

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading, Segment, SegmentList, Vector2D

from lane_controller.controller import PurePursuitLaneController

class Color(enum.Enum):
    WHITE = Segment.WHITE
    YELLOW = Segment.YELLOW
    RED = Segment.RED
class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocitie.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Add the node parameters to the parameters dictionary
        self.params = dict()
        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
                                           Twist2DStamped,
                                           queue_size=1,
                                           dt_topic_type=TopicType.CONTROL)

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber("~lane_pose",
                                                 LanePose,
                                                 self.cbLanePoses,
                                                 queue_size=1)

        self.sub_segments = rospy.Subscriber("/agent/lane_filter_node/seglist_filtered",
                                                 SegmentList,
                                                 self.cbSegList,
                                                 queue_size=1)
        
        #self.log("Initialized!")
        
        # Velocity and Omega
        self.v = 0
        self.omega = 0
        # Velocity and Omega after update
        self.av = 0
        self.ao = 0
        
        # The Lookahead distance for Pure Pursuit
        self.lookahead_dist = 1.0
        
        #Length of the dictionary buffer containing of segment points based on their color in a deque.
        self.blength = 150
        self.seg_points = collections.defaultdict(lambda: collections.deque(maxlen=self.blength))
        
        # Variable to control the velocity of the bot
        self.max_velocity = 0.28
        
        #Setting a timer as suggested by the TA for the car command
        self.d_t = 0.5
        self.cc_timer = rospy.Timer(rospy.Duration.from_sec(self.d_t), self.car_command)
        
        # An offset parameter which is quite useful specially at curves
        self.offset = 0.15
        
        # Thread lock to access points deque
        self.lock_for_points = Lock()
        
        # Target(Follow) point
        self.target = np.asarray([0,0])
        
        # Target(Follow) point message
        self.target_msg = Vector2D()
        
        self.controller = PurePursuitLaneController(self.params)
        
        self.car_control_msg = Twist2DStamped()
        
        # List of White points on the right of the yellow points
        self.wpr = collections.deque(maxlen = self.blength)
        
        # Flag for White points
        self.wflag = 0
        
        # List of Yellow points in the middle
        self.ypm = collections.deque(maxlen = self.blength)
        
        # Flag for White points
        self.yflag = 0

    def cbLanePoses(self, input_pose_msg):
        """Callback receiving pose messages from multiple topics.

        If the source of the message corresponds with the current wanted pose source, it computes a control command.

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        self.pose_msg = input_pose_msg
        if self.pose_msg.phi < 0:
            print("LANE POSE NEGATIVE: ",self.pose_msg.phi)
            print("LANE POSE NEGATIVE D: ",self.pose_msg.d)
        else:
            print("LANE POSE POSITIVE: ",self.pose_msg.phi)
            print("LANE POSE POSITIVE D: ",self.pose_msg.d)
        car_control_msg1 = Twist2DStamped()
        car_control_msg1.header = self.pose_msg.header

        # TODO This needs to get changed
        #car_control_msg1.v = self.target[0] #np.random.random_sample() #self.av
        #car_control_msg1.omega = self.target[1] #-np.random.random_sample() #self.ao
        if self.target[0] != 0 or self.target[1] != 0:
            car_control_msg1.v,car_control_msg1.omega = self.controller.pure_pursuit()
        else:
            car_control_msg1.v = 0.4
            car_control_msg1.omega = 0.05
        #print("CONTROL COMMAND: ", car_control_msg1)
        self.publishCmd(car_control_msg1)

    def cbSegList(self, input_segments):
        """Callback receiving segment messages

        Args:
            input_segments (:obj:`SegmentList`): Message containing information about detected segments.
        """
        self.header = input_segments.header
        segments = input_segments.segments
        #print("LINE SEGMENTS: ", segments)
        for i, segment in enumerate(segments):
            color = Color(segment.color)
            assert color in [Color.RED, Color.YELLOW, Color.WHITE]
            with self.lock_for_points:
                self.seg_points[color].extend(segment.points)
    
    def car_command(self, timer):
        if not self.points_are_available(Color.YELLOW) and not self.points_are_available(Color.WHITE):
            #self.logwarn("THERE ARE NO POINTS AVAILABLE")
            if self.v == 0 and self.omega == 0:
                #self.logwarn("DUCKIEBOT CANT DETECT ANY SEGMENTS AND IS THEREFORE STAGNANT.")
                self.car_control_msg.v = 0.05
                self.car_control_msg.omega = 0
                #print("COMMAND 1 EXECUTED")
            else:
                #self.logwarn("SEGMENTS NOT DETECTED(NOT ABLE TO SEE)")
                self.car_control_msg.v = 0
                self.car_control_msg.omega = 0
                #print("COMMAND 2 EXECUTED")
        
        else:
            """
            if self.counting_points(Color.YELLOW) != 0:
                #self.yellow_mid()
                #if len(self.ypm) != 0:
                #self.right_white()
                yellow_centroid = self.calculate_centroid(Color.YELLOW)
                #self.ypm.clear()
                #self.yflag = 0
                #self.wflag = 1
                white_centroid = self.calculate_centroid(Color.WHITE)
                self.wpr.clear()
                self.target = (white_centroid + yellow_centroid) / 2
            else:
                #self.right_white()
                #self.wflag = 1
                print("TRUE 1")
                white_centroid = self.calculate_centroid(Color.WHITE)
                self.target = white_centroid
                self.target[1] += self.offset # shift to the left.
                #yellow_centroid = self.calculate_centroid(Color.YELLOW)
                #white_centroid = self.calculate_centroid(Color.WHITE)
                #self.target = (white_centroid + yellow_centroid) / 2
                #self.target[1] += 0.1
            """
            if self.counting_points(Color.YELLOW) !=0 and self.pose_msg.phi > 0: 
                #> self.counting_points(Color.WHITE)
                yellow_centroid = self.calculate_centroid(Color.YELLOW)
                self.target = yellow_centroid
                print("CONDITION 1")
                #print("TARGET 1Y: ",self.target)
                self.target[1] -= self.offset # moving the follow(target) point to the right.
                #print("CONDITION TRUE")
            elif self.counting_points(Color.WHITE) !=0 and self.pose_msg.phi < 0:
                white_centroid = self.calculate_centroid(Color.WHITE)
                self.target = white_centroid
                print("CONDITION 2:")
                #print("TARGET 1W: ",self.target)
                self.target[1] += self.offset # moving the follow(target) point to the left.
                #print("PRINTING TARGET: ",target)
            else:
                print("CONDITION 3")
                self.target[1] = self.target[1] # same follow(target) point.
        #print("SEGMENT COLOR WHITE: ", Segment.WHITE)
        #print("SEGMENT COLOR YELLOW: ", Segment.YELLOW)
        #print("SEGMENT COLOR RED: ", Segment.RED)
        #print("MY SEGMENT LIST: ", self.seg_points )
        print("PRINTING TARGET: ",self.target)
        print("YELLOW LINE SEGMENTS: ", self.counting_points(Color.YELLOW))
        print("WHITE LINE SEGMENTS: ", self.counting_points(Color.WHITE))
        print("RIGHT WHITE LINE SEGMENTS: ", len(self.wpr))
        print("RIGHT YELLOW LINE SEGMENTS: ", len(self.ypm))
        #self.target_msg = Vector2D()
        self.target_msg.x = self.target[0]
        self.target_msg.y = self.target[1]
        
        #print("TARGET 2: ",self.target)
        
        if self.target[0] != 0 or self.target[1] != 0:
            self.cbParametersChanged()
        
        #target_dot_product =  self.target.dot(self.target)
        """hypothenuse = np.sqrt(self.target.dot(self.target))
        sin_alpha = self.target[1] / hypothenuse
        min_speed = 0.1
        max_speed = 1.0
        v = self.max_velocity * (1 - abs(sin_alpha))
        v = np.clip(v, min_speed, max_speed)
        omega = 2 * sin_alpha / self.lookahead_dist"""
        #self.car_control_msg.v,self.car_control_msg.omega = self.controller.pure_pursuit()
        #self.ao = omega
        # Emptying all the stored points in the deque
        self.empty_all_points()
        
    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)


    def cbParametersChanged(self):
        """Updates parameters in the controller object."""
        self.params["lookahead_dist"] = self.lookahead_dist
        self.params["max_velocity"] = self.max_velocity
        self.params["target"] = self.target
        self.controller.update_parameters(self.params)
    
    def right_white(self):
        #self.wpr.clear()
        with self.lock_for_points:
            yellow_points = self.seg_points[Color.YELLOW]
            white_points = self.seg_points[Color.WHITE]
            for p in white_points:
                #for q in yellow_points:
                    #if p.x > q.x:
                if p.y < 0:
                    self.wpr.append(p)
                    self.wflag = 1
    
    def yellow_mid(self):
        #self.ypm.clear()
        with self.lock_for_points:
            yellow_points = self.seg_points[Color.YELLOW]
            #white_points = self.seg_points[Color.WHITE]
            for p in yellow_points: #self.seg_points[Color.YELLOW]:
                if p.y > 0:
                    self.ypm.append(p)
                    self.yflag = 1
    
    def calculate_centroid(self, color=None):
        with self.lock_for_points:
            if color is None:
                #self.logdebug("Averaging points of all colors")
                points_to_average = list(itertools.chain(*self.seg_points.values()))
            
            elif self.wflag == 1 and color == Color.WHITE:
                #self.logdebug("Averaging points of color {}".format(color))
                print("TRUE 22")
                points_to_average = self.wpr
                self.wflag = 0
            
            elif self.yflag == 1 and color == Color.YELLOW:
                #self.logdebug("Averaging points of color {}".format(color))
                print("TRUE 23")
                points_to_average = self.ypm
                self.yflag = 0
            
            else:
                #self.logdebug("Averaging points of color {}".format(color))
                points_to_average = self.seg_points[color]
                #print("POINTS TO AVERAGE: ", self.seg_points[color])
            x = np.mean([p.x for p in points_to_average])
            y = np.mean([p.y for p in points_to_average])
        return np.asarray([x, y])
            
    def points_are_available(self, color=None):
        return self.counting_points(color) != 0

    def counting_points(self, color=None):
        with self.lock_for_points:
            if color is None:
                return sum(len(values) for values in self.seg_points.values())
            return len(self.seg_points[color])

    def empty_all_points(self):
        with self.lock_for_points:
            for color, point_list in self.seg_points.items():
                point_list.clear()

if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
