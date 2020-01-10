from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sin, cos

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveBaseClient(object):
    """
    Class to move the fetch base to desired position
    """
    def __init__(self, rospy):
        # Constants for the class
        ROSTOPIC_NAME = "move_base"
        # Initialization
        self.rospy = rospy
        self.client = SimpleActionClient(ROSTOPIC_NAME, MoveBaseAction)
        self.rospy.loginfo("Waiting for /move_base ....")
        self.client.wait_for_server()
        self.rospy.loginfo("Connected to /move_base")
    
    def goto2d(self, x, y, theta = 0.0, frame = "map"):
        """
        Structure:
        target_pose: 
            header: 
                seq: 0
                stamp: 
                    secs: 0
                    nsecs:         0
                frame_id: ''
            pose: 
                position: 
                x: 0.0
                y: 0.0
                z: 0.0
                orientation: 
                x: 0.0
                y: 0.0
                z: 0.0
                w: 0.0
        """
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = self.rospy.Time.now()

        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        #self.client.send_goal_and_wait(move_goal,execute_timeout=rospy.Duration(10,0), preempt_timeout=rospy.Duration(20,0))

    def goto3d(self):
        """
        Function not implemented
        """
        pass

class FollowTrajectoryClient(object):
    """
    Class to make modules (arm|torso) follow trajectory
    """
    def __init__(self, rospy, name, joint_names):
        self.rospy =rospy
        self.client = SimpleActionClient("%s/follow_joint_trajectory" % name, FollowJointTrajectoryAction)
        self.rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names
    
    def move_to(self, positions, duration = 5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = self.rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

"""
header: 
  seq: 34721
  stamp: 
    secs: 347
    nsecs: 385000000
  frame_id: ''
name: [l_wheel_joint, r_wheel_joint, torso_lift_joint, bellows_joint, head_pan_joint, head_tilt_joint,
  shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint,
  forearm_roll_joint, wrist_flex_joint, wrist_roll_joint, l_gripper_finger_joint,
  r_gripper_finger_joint]
position: [0.6575765528403394, 0.6633769963944403, -2.5285273806677765e-08, 0.006637594469476392, 6.100760741034605e-07, 0.002394657050917459, 1.3199986769031682, 1.3999858557591898, -0.19998891622343873, 1.7199619190973472, 2.3394379526919806e-06, 1.6600080895044478, -4.986127422057507e-07, 0.05004753546722228, 0.05007338188153915]
velocity: [-9.239985103739735e-06, 1.2981935615118246e-05, 6.529871050435516e-05, -8.266088849861341e-05, 4.8615047559163355e-05, -0.00012089114021054698, -7.886969840740254e-05, 0.0002965065450734016, -6.867637883917665e-05, -0.0007401140075827056, 0.00012545333811984536, 0.00056759965629606, -8.17132493188857e-06, 0.008849513786460626, 0.00881208537447555]
effort: [0.0, 0.0, 0.0, -1.6573352558157923, -9.967039941474055e-05, -0.07147703810683885, 0.02906142288676407, 11.662826725052597, -0.6210204141159461, 18.572922844996, 0.18682267578223843, -0.4947573997314005, 0.005603341065414881, -9.974153585683132, -10.025846414316868]
"""
class MoveArmClient(object):
    """
    Class to move the robotic arm
    """
    def __init__(self):
        # Constants for the class
        ROSTOPIC_NAME = "move_base"
        # Initialization
        self.rospy = rospy
        self.client = SimpleActionClient(ROSTOPIC_NAME, MoveBaseAction)
        self.rospy.loginfo("Waiting for /move_base ....")
        self.client.wait_for_server()
        self.rospy.loginfo("Connected to /move_base")

