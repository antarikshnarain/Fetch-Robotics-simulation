import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from random import randint

if __name__ == "__main__":
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('robot_motion')
    r = rospy.Rate(2) # 10hz
    cmd_vel = Twist()
    while not rospy.is_shutdown():
        #pub.publish("hello world")
        a1 = randint(0,3)
        a2 = randint(0,3)
        if a1 == 1:
            cmd_vel.linear.x = randint(-30,30)
            cmd_vel.linear.y = cmd_vel.linear.z = 0
        elif a1 == 2:
            cmd_vel.linear.y = randint(-30,30)
            cmd_vel.linear.x = cmd_vel.linear.z = 0
        elif a1 == 3:
            cmd_vel.linear.z = randint(-30,30)
            cmd_vel.linear.y = cmd_vel.linear.x = 0
        
        if a2 == 1:
            cmd_vel.angular.x = randint(-90,90)
            cmd_vel.angular.y = cmd_vel.angular.z = 0
        elif a2 == 2:
            cmd_vel.angular.y = randint(-90,90)
            cmd_vel.angular.x = cmd_vel.angular.z = 0
        elif a2 == 3:
            cmd_vel.angular.z = randint(-90,90)
            cmd_vel.angular.x = cmd_vel.angular.y = 0
        
        pub.publish(cmd_vel)
        rospy.loginfo(cmd_vel)
        r.sleep()
        