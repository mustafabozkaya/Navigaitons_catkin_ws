#!/usr/bin/env python
import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('my_moving_carrot_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(5.0)
    turning_speed_rate = 0.1
    while not rospy.is_shutdown():
        t = (rospy.Time.now().to_sec() * math.pi)*turning_speed_rate
        # Map to only one turn maximum [0,2*pi)
        rad_var = t % (2*math.pi)
        quaternion = tf.transformations.quaternion_from_euler(
            0.0, 0.0, -5*rad_var)
        quaternion_x = quaternion[0]
        quaternion_y = quaternion[1]
        quaternion_z = quaternion[2]
        quaternion_w = quaternion[3]
        br.sendTransform((1.0 * math.sin(rad_var), 1.0 * math.cos(rad_var), 0.0),
                         (quaternion_x, quaternion_y, quaternion_z, quaternion_w),
                         rospy.Time.now(),
                         "moving_carrot",
                         "world")
        rate.sleep()
