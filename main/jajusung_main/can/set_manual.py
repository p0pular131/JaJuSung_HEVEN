#!/usr/bin/env python3
import can
import cantools
import rospy

class CanManual():
    def __init__(self):
        # CAN Database
        self.db = cantools.db.load_file('/home/popular/catkin_ws/src/JaJuSung_HEVEN/main/jajusung_main/can/auds_drive.dbc')
        self.db_steer_msg = self.db.get_message_by_name('TargetSteeringAngle')
        self.db_MorA_msg = self.db.get_message_by_name('ManualAutoMode')
        self.db_Cmd_msg = self.db.get_message_by_name('UPPER_CMD_DATA')
        self.mutex_lock = False
        self.motor_alive_count = 0
        rospy.init_node("manualNode", anonymous=False)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
            
    def update(self):
        # Transmit once
        if self.mutex_lock == False:
            self.mutex_lock = True
            with can.interface.Bus(channel='PCAN_USBBUS1', bustype='pcan', bitrate=500000) as bus:
                message_name = "UPPER_CMD_DATA"
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
        CanManual()
    except rospy.ROSInterruptException:
        pass