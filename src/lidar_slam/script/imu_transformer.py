#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu

class IMUTransformer(object):
    def __init__(self):
        # ros node
        rospy.init_node("rotate_IMU_to_global")

        # tf
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        (trans, rot) = self.get_transform('global', 'imu_link')

        # rotation
        rot_matrix = tf.transformations.quaternion_matrix(
            [rot[0], rot[1], rot[2], rot[3]]
        )
        # translation
        rot_matrix[0, 3] = 0
        rot_matrix[1, 3] = 0
        rot_matrix[2, 3] = 0

        # remove trunction error
        rot_matrix = np.round(rot_matrix, 6)

        self.global_T_imu = rot_matrix
        self.imu_T_global = rot_matrix.T
        rospy.loginfo("The rotation matrix from IMU to global is:\n %s", self.imu_T_global)

        rospy.Subscriber("/imu/data", Imu, self.msg_callback)
        self.pub = rospy.Publisher("/imu/data/global", Imu, queue_size=10)

    def msg_callback(self, msg):
        imu_msg_global = Imu()
        imu_msg_global.header = msg.header
        imu_msg_global.header.frame_id = "global" # new frame_id
        
        # orientation
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w
        qt_imu = tf.transformations.quaternion_matrix([q_x, q_y, q_z, q_w]) # [w, x, y, z]
        qt_imu_global = self.imu_T_global.dot(qt_imu)
        qx, qy, qz, qw = tf.transformations.quaternion_from_matrix(qt_imu_global)
        imu_msg_global.orientation.x = qx
        imu_msg_global.orientation.y = qy
        imu_msg_global.orientation.z = qz
        imu_msg_global.orientation.w = qw        

        orientation_covariance = abs(self.imu_T_global[:3, :3].dot(
                        np.array(msg.orientation_covariance).reshape((3,3))))
        imu_msg_global.orientation_covariance = orientation_covariance.flatten().tolist()

        # angualar velocity
        angular_velocity = self.imu_T_global[:3, :3].dot([msg.angular_velocity.x, 
                                                  msg.angular_velocity.y,
                                                  msg.angular_velocity.z])
        imu_msg_global.angular_velocity.x = angular_velocity[0]
        imu_msg_global.angular_velocity.y = angular_velocity[1]
        imu_msg_global.angular_velocity.z = angular_velocity[2]
        
        angular_velocity_covariance = abs(self.imu_T_global[:3, :3].dot(
                        np.array(msg.angular_velocity_covariance).reshape((3,3))))
        imu_msg_global.angular_velocity_covariance = angular_velocity_covariance.flatten().tolist()
        
        # linear acceleration
        linear_acceleration = self.imu_T_global[:3, :3].dot([msg.linear_acceleration.x, 
                                                     msg.linear_acceleration.y,
                                                     msg.linear_acceleration.z])
        imu_msg_global.linear_acceleration.x = linear_acceleration[0]
        imu_msg_global.linear_acceleration.y = linear_acceleration[1]
        imu_msg_global.linear_acceleration.z = linear_acceleration[2]

        linear_acceleration_covariance = abs(self.imu_T_global[:3, :3].dot(
                        np.array(msg.linear_acceleration_covariance).reshape((3,3))))
        imu_msg_global.linear_acceleration_covariance = linear_acceleration_covariance.flatten().tolist()

        # publish
        self.pub.publish(imu_msg_global)       

    def get_transform(self, target_frame, source_frame):
        success = False
        while not rospy.is_shutdown() and not success:
            success, transform = self._lookup_transformation(
                self.tf_listener, target_frame, source_frame
            )
            if not success:
                rospy.logwarn(
                    "lookupTransform from %s to %s failed"
                    % (target_frame, source_frame)
                )
            rospy.sleep(0.1)

        return transform

    def _lookup_transformation(self, listener, target_frame, source_frame):
        success = False
        transform = None
        try:
            transform = listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0)
            )
            success = True
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            pass
    
        return success, transform

if __name__ == "__main__":
    handler = IMUTransformer()
    rospy.spin()
