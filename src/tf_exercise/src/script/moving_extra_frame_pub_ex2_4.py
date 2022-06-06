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

        my_quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,rad_var)

        br.sendTransform((0.0, 0.0, 0.0),
                         (my_quaternion[0], my_quaternion[1], my_quaternion[2], my_quaternion[3]),
                         rospy.Time.now(),
                         "moving_carrot_ex2_4",
                         "coke_can")
        rate.sleep()