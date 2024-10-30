#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Imu
import asyncio
from device_model import DeviceModel
import bleak

# MAC 주소 설정
TARGET_MAC_ADDRESS = "DF:57:FE:B4:2A:21"

# Callback to update and publish data
def updateData(device_model):
    imu_msg = Imu()
    
    # Populate IMU message fields
    imu_msg.linear_acceleration.x = device_model.get("AccX") or 0.0
    imu_msg.linear_acceleration.y = device_model.get("AccY") or 0.0
    imu_msg.linear_acceleration.z = device_model.get("AccZ") or 0.0
    imu_msg.angular_velocity.x = device_model.get("AsX") or 0.0
    imu_msg.angular_velocity.y = device_model.get("AsY") or 0.0
    imu_msg.angular_velocity.z = device_model.get("AsZ") or 0.0
    
    # Quaternion values
    imu_msg.orientation.x = device_model.get("Q1") or 0.0
    imu_msg.orientation.y = device_model.get("Q2") or 0.0
    imu_msg.orientation.z = device_model.get("Q3") or 0.0
    imu_msg.orientation.w = device_model.get("Q0") or 1.0  # Typically Q0 is the w component

    # Publish the IMU message
    imu_publisher.publish(imu_msg)

async def main():
    # ROS node and publisher initialization
    rospy.init_node('imu_publisher', anonymous=True)
    global imu_publisher
    imu_publisher = rospy.Publisher('/imu', Imu, queue_size=10)
    
    # Bluetooth device search by MAC address
    target_device = await bleak.BleakScanner.find_device_by_address(TARGET_MAC_ADDRESS, timeout=20.0)

    if not target_device:
        rospy.logwarn("No matching Bluetooth device found!")
        return
    
    device = DeviceModel("MyBle5.0", target_device, updateData)
    await device.openDevice()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
        pass
