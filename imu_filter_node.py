#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler


class IMUFilter:
    def __init__(self):
        rospy.init_node('imu_filter', anonymous=True)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.pub_filtered_imu = rospy.Publisher("/imu/filtered", Imu, queue_size=10)
        self.last_time = rospy.Time.now()
        self.last_orientation = [0.0, 0.0, 0.0, 0.0]

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        orientation = data.orientation
        orientation_quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]
        angular_velocity = data.angular_velocity

        
        roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)

        
        delta_orientation = quaternion_multiply(
            self.last_orientation,
            quaternion_from_euler(
                angular_velocity.x * dt,
                angular_velocity.y * dt,
                angular_velocity.z * dt
            )
        )
        self.last_orientation = delta_orientation

        # Publish filtered IMU data
        filtered_imu = Imu()
        filtered_imu.header = data.header
        filtered_imu.orientation.x = delta_orientation[0]
        filtered_imu.orientation.y = delta_orientation[1]
        filtered_imu.orientation.z = delta_orientation[2]
        filtered_imu.orientation.w = delta_orientation[3]
        self.pub_filtered_imu.publish(filtered_imu)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        imu_filter = IMUFilter()
        imu_filter.run()
    except rospy.ROSInterruptException:
        pass