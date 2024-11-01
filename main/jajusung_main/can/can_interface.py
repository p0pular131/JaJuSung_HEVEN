#!/usr/bin/env python3
import can
import cantools
import rospy
from random import randint
from jajusung_main.msg import HevenCtrlCmd

class VCUCAN():
    def __init__(self):
        self.brake = 0
        self.steering = 0
        self.velocity = 0
        self.mutex_lock = False
        self.motor_alive_count = 0

        # CAN Database
        self.db = cantools.db.load_file('/home/heven/jajusung_ws/src/JaJuSung_HEVEN/main/jajusung_main/can/auds_drive.dbc')
        self.db_steer_msg = self.db.get_message_by_name('TargetSteeringAngle')
        self.db_MorA_msg = self.db.get_message_by_name('ManualAutoMode')
        self.db_Cmd_msg = self.db.get_message_by_name('UPPER_CMD_DATA')
        self.bus = can.interface.Bus(channel='PCAN_USBBUS1', bustype='pcan', bitrate=500000)
        rospy.init_node("steering_can_node")
        rospy.Subscriber("/drive", HevenCtrlCmd, self.cmd_callback)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def cmd_callback(self, data):
        self.steering = data.steering
        self.brake = data.brake
        self.velocity = data.velocity

    def update(self):
        # Transmit once
        if not self.mutex_lock:
            self.mutex_lock = True

            # ========================= steer can =====================================
            # Manual/Auto 모드 메시지 전송
            MorA_dict = {'Mode': 1}
            MorA_data = self.db_MorA_msg.encode(MorA_dict)
            message = can.Message(arbitration_id=self.db_MorA_msg.frame_id, data=MorA_data)
            self.bus.send(message)
            # Steering angle 메시지 전송
            if self.steering > 39.0 : 
                self.steering = 39.0
            elif self.steering < -39.0 :
                self.steering = -39.0
            rospy.loginfo("Curr steer : %d", self.steering)
            steering_dict = {'TargetSteeringAngle': -self.steering * 10000}
            steer_data = self.db_steer_msg.encode(steering_dict)
            message = can.Message(arbitration_id=self.db_steer_msg.frame_id, data=steer_data)
            self.bus.send(message)
            # ========================= vcu can ======================================
            Cmd_dict = {
                'cmd_estop_toggle': self.brake,  # 1 -> estop, 0 -> normal
                'cmd_motor_speed_limit': 3,  # 3 -> 1x, 2 -> 1/2x, 1 -> 1/10x, 0 -> 1/50x
                'cmd_motor_torq_value': self.velocity,  # 0 ~ 3200 
                'cmd_enable_': 1,  # 1 -> auto, 0 -> manual
                'cmd_motor_torq_direction': 1,
                'motor_alive_count': self.motor_alive_count
            }< -39.0 :

            # Manual Converter For Start Line
            if self.brake == -1 :
                Cmd_dict = {
                    'cmd_estop_toggle' : 0,          # 1 -> estop, 0 -> normal
                    'cmd_motor_speed_limit' : 3,     # 3 -> 1x, 2 -> 1/2x, 1 -> 1/10x, 0 -> 1/50x
                    'cmd_motor_torq_value' : 0,      # 0 ~ 3200
                    'cmd_enable_' : 0,               # 1 -> auto, 0 -> manual
                    'cmd_motor_torq_direction' : 1,
                    'motor_alive_count' : self.motor_alive_count
                }

            Cmd_data = self.db_Cmd_msg.encode(Cmd_dict)
            message = can.Message(arbitration_id=self.db_Cmd_msg.frame_id, data=Cmd_data)
            self.bus.send(message)
            self.motor_alive_count = (self.motor_alive_count + 1) % 256

            self.mutex_lock = False
            rospy.loginfo("Curr vel : %d",self.velocity)
            rospy.loginfo("Braking : %d", self.brake)
            rospy.loginfo("Send message.")


if __name__ == "__main__":
    try:
        VCUCAN()
    except rospy.ROSInterruptException:
        pass