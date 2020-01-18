from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sin, cos

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#from moveit_python import MoveGroupInterface
from move_group_interface import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

# Point the head using controller
class PointHeadClient(object):

    def __init__(self, rospy):
        self.rospy = rospy
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        self.rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Move base using navigation stack
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

# Move Torso using controller
class FollowTrajectoryClient(object):
    """
    Class to make modules (torso) follow trajectory
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

# Move Gripper using controller
class GraspingClient(object):

    def __init__(self,rospy):
        # self.scene = PlanningSceneInterface("base_link")
        # self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.rospy = rospy
        self.move_group = MoveGroupInterface("arm", "base_link")

        # find_topic = "basic_grasping_perception/find_objects"
        # rospy.loginfo("Waiting for %s..." % find_topic)
        # self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        # self.find_client.wait_for_server()
    
    def tuck(self):
        self.rospy.loginfo("Tucking ...")
        #joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
        #         "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        #pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        #pose = [1, 1, 0, 1.22, 1.0, 1.0, 0.0]

        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
        "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint", "r_gripper_finger_joint",
        "l_gripper_finger_joint"]
        pose = [1.32, 1.42, 0.0, 1.70, 
        0.0, 1.62, 0.0, 0.0, 
        0.0]
        while not self.rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

        self.rospy.loginfo("Tuck Done!")


class MoveFetch(object):

    def __init__(self, rospy):
        self.rospy = rospy
        self.rospy.loginfo("In Move Fetch Calss init...")

        # Init Torso Action
        self.torso_action = FollowTrajectoryClient( self.rospy
            "torso_controller", ["torso_lift_joint"])

        # Gripper Action
        self.gripper_action = GripperActionClient(self.rospy)

        # Point Head action
        self.head_action = PointHeadClient(self.rospy)

        # MOVEIT
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")

        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")

        rospy.logwarn("self.group TYPE==>"+str(type(self.group)))

        rospy.loginfo("FETCH ready to move!")

    def move_manager(self, pose_requested, joints_array_requested, movement_type_requested):

        success = False

        if movement_type_requested == "TCP":
            success = self.ee_traj(pose_requested)
        elif movement_type_requested == "JOINTS":
            success = self.joint_traj(joints_array_requested)
        elif movement_type_requested == "TORSO":
            torso_height = joints_array_requested[0]
            success = self.move_torso(torso_height)
        elif movement_type_requested == "HEAD":
            # Roll, Pitch and Yaw
            XYZ = [joints_array_requested[0],
                   joints_array_requested[1],
                   joints_array_requested[2]]
            success = self.move_head_point(XYZ)
        elif movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]

            success = self.move_gripper(gripper_x, max_effort)
        else:
            self.rospy.logerr("Asked for non supported movement type==>" +
                         str(movement_type_requested))

        return success

    def ee_traj(self, pose):

        pose_frame = self.group.get_pose_reference_frame()

        if pose_frame != "base_link":
            new_reference_frame = "base_link"
            self.group.set_pose_reference_frame(new_reference_frame)

            pose_frame = self.group.get_pose_reference_frame()

        else:
            pass

        self.group.set_pose_target(pose)

        result = self.execute_trajectory()

        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        self.rospy.logdebug("Group Vars:")
        self.rospy.logdebug(self.group_variable_values)
        self.rospy.logdebug("Point:")
        self.rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory()

        return result

    def move_torso(self, torso_height):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.torso_action.move_to([torso_height, ])

        return result

    def move_gripper(self, gripper_x, max_effort):
        """
        Moves the gripper to given pose
        """
        result = self.gripper_action.move_gripper(gripper_x, max_effort)

        return result

    def move_head_point(self, XYZ, frame="base_link"):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.head_action.look_at(XYZ[0], XYZ[1], XYZ[2], frame)

        return result

    def execute_trajectory(self):

        self.plan = self.group.plan()
        result = self.group.go(wait=True)

        return result

    def ee_pose(self):

        gripper_pose = self.group.get_current_pose()

        self.rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose

    def ee_rpy(self, request):

        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy


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
"""
header: 
  seq: 990
  stamp: 
    secs: 1578778444
    nsecs: 890126705
  frame_id: ''
name: [r_wheel_joint, l_wheel_joint, torso_lift_joint, head_pan_joint, head_tilt_joint,
  shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint,
  forearm_roll_joint, wrist_flex_joint, wrist_roll_joint, r_gripper_finger_joint,
  l_gripper_finger_joint, bellows_joint]
position: [0.0, 0.0, 0.0, 0.0, -0.00020200000000003548, 0.0, -0.00022770000000016388, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []

"""
class MoveArmClient(object):
    """
    Class to move the robotic arm
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

