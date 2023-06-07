#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('turtlebot_location_listener')

    # Create a tf listener
    listener = tf.TransformListener()

    # Wait for the first transform to become available
    listener.waitForTransform('/gazebo/get_model_state', '/mobile_base', rospy.Time(), rospy.Duration(1.0))

    try:
        while not rospy.is_shutdown():
            # Get the latest transform between the 'map' and 'base_link' frames
            (translation, rotation) = listener.lookupTransform('/gazebo/get_model_state', '/mobile_base', rospy.Time(0))

            # Extract the position (x, y, z)
            x = translation[0]
            y = translation[1]
            z = translation[2]

            print('TurtleBot Location - x: ',x,'y:',y, 'z:',z)

            # Sleep for a short duration
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass