# odom_to_tf_ros2

A simple ros2 package (node) that reads an odom topic and generates the equivalent tf connection (transformation). It also provides options to override frame names, or just use the ones in the original odom topic.

# Parameters

- `frame_id`: The parent frame id to override. (Leave empty to use the one from the odom topic)

- `child_frame_id`: The child frame id to override. (Leave empty to use the one from the odom topic)


- `odom_topic`: The name of the nav\_msgs/Odometry topic to subscribe to.
