#!/usr/bin/env python3
import can
import cantools
import rospy
from jajusung_main.msg import HevenCtrlCmd

class VCUCAN():
    def __init__(self):
        self.brake = 0
        self.steering = 0
        self.velocity = 0
        self.mutex_lock = False
        self.motor_alive_count = 0
        # CAN Database
        self.db = cantools.db.load_file('/home/popular/catkin_ws/src/heven_ev_auto/heven_ev_auto/assets/auds_drive.dbc')
        self.db_steer_msg = self.db.get_message_by_name('TargetSteeringAngle')
        self.db_MorA_msg = self.db.get_message_by_name('ManualAutoMode')
        self.db_Cmd_msg = self.db.get_message_by_name('UPPER_CMD_DATA')
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
        self.update()

    def update(self):
        # Transmit once
        if self.mutex_lock == False:
            self.mutex_lock = True
        #========================= steer can =====================================
            with can.interface.Bus(channel='PCAN_USBBUS1', bustype='pcan', bitrate=500000) as bus:
                message_name = "ManualAutoMode"
                MorA_dict = {
                    'Mode' : 1
                }
                MorA_data = self.db_MorA_msg.encode(MorA_dict)
                message = can.Message(arbitration_id = self.db_MorA_msg.frame_id, data = MorA_data)
                bus.send(message)
            with can.interface.Bus(channel='PCAN_USBBUS1', bustype='pcan', bitrate=500000) as bus:
                message_name = "TargetSteeringAngle"
                rospy.loginfo("Curr steer : %d", self.steering)
                steering_dict = {
                    'TargetSteeringAngle' : -int(self.steering) * 10000
                }
                steer_data = self.db_steer_msg.encode(steering_dict)
                message = can.Message(arbitration_id = self.db_steer_msg.frame_id, data = steer_data)
                bus.send(message)
        #========================================================================
        #========================= vcu can ======================================
            with can.interface.Bus(channel='PCAN_USBBUS1', bustype='pcan', bitrate=500000) as bus:
                message_name = "UPPER_CMD_DATA"
                # Cmd_dict = {
                #     'cmd_estop_toggle' : self.brake,          # 1 -> estop, 0 -> normal
                #     'cmd_motor_speed_limit' : 3,     # 3 -> 1x, 2 -> 1/2x, 1 -> 1/10x, 0 -> 1/50x
                #     'cmd_motor_torq_value' : 200,      # 0 ~ 3200
                #     'cmd_enable_' : 1,               # 1 -> auto, 0 -> manual
                #     'cmd_motor_torq_direction' : 1,
                #     'motor_alive_count' : self.motor_alive_count
                # }
                # estop 풀고 메뉴얼로 변환시
                # estop 0으로 변경, tork 0으로 변경, manucal로 변경
                Cmd_dict = {
                    'cmd_estop_toggle' : 0,          # 1 -> estop, 0 -> normal
                    'cmd_motor_speed_limit' : 3,     # 3 -> 1x, 2 -> 1/2x, 1 -> 1/10x, 0 -> 1/50x
                    'cmd_motor_torq_value' : 0,      # 0 ~ 3200
                    'cmd_enable_' : 0,               # 1 -> auto, 0 -> manual
                    'cmd_motor_torq_direction' : 1,
                    'motor_alive_count' : self.motor_alive_count
                }
                Cmd_data = self.db_Cmd_msg.encode(Cmd_dict)
                message = can.Message(arbitration_id = self.db_Cmd_msg.frame_id, data = Cmd_data)
                bus.send(message)
                self.motor_alive_count = (self.motor_alive_count + 1) % 256
        #========================================================================
            self.mutex_lock = False
            rospy.loginfo("Send message.")
if __name__ == "__main__":
    try:
        VCUCAN()
    except rospy.ROSInterruptException:
        pass