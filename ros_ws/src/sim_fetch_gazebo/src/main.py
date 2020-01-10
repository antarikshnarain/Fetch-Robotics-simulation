import rospy

from fetch_control import MoveBaseClient, FollowTrajectoryClient

if __name__ == "__main__":
    rospy.init_node("FetchControl")

    while not rospy.Time.now():
        pass
    
    move_base = MoveBaseClient(rospy)
    torso_action = FollowTrajectoryClient(rospy, "torso_controller", ["torso_lift_joint"])

    rospy.loginfo("Moving the base ....")
    # move_base.goto2d(0,0)
    # rospy.loginfo("2.250, 3.118, 0.0")
    # move_base.goto2d(2.250, 3.118, 0.0)
    # rospy.loginfo("2.750, 3.118, 0.0")
    # move_base.goto2d(2.750, 3.118, 0.0)
    rospy.loginfo("2.250, 3.118, 2.0")
    move_base.goto2d(2.250, 3.118, 2.0)

    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])
    rospy.loginfo("Lowering torso...")
    torso_action.move_to([0.0, ])

    rospy.loginfo("----------Exection Complete------------")
