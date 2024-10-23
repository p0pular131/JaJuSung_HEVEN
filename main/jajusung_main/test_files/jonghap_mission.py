#!/usr/bin/env python3

import rospy
from steering_control import SteeringCtrl
from jajusung_main.msg import HevenCtrlCmd

# Default speed (km/h)
DEFAULT_VEL=8


class JonghapMission:
    def __init__(self):
        rospy.init_node("jonghap_node")
        self.steering_controller = SteeringCtrl(DEFAULT_VEL)
        self.rate = rospy.Rate(10)

        self.msg_pub = rospy.Publisher("/drive", HevenCtrlCmd, queue_size=1)

        rospy.loginfo("Jonghap node starts...")
        rospy.on_shutdown(self.term)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()

    def term(self):
        m = HevenCtrlCmd()
        m.velocity = 0
        m.steering = 0
        m.brake = 0

        self.msg_pub.publish(m)

    def main(self):
        # Steering
        control_msg = self.steering_controller.main()
        
        self.msg_pub.publish(control_msg)


if __name__ == "__main__":
    try:
        JonghapMission()
    except rospy.ROSInterruptException:
        pass