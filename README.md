# odom_to_tf_ros2

A simple ros2 package (node) that reads an odom topic and generates the equivalent tf connection (transformation). It also provides options to override frame names, or just use the ones in the original odom topic.

# Parameters

- `frame_id`: The parent frame id to override. (Leave empty to use the one from the odom topic)

- `child_frame_id`: The child frame id to override. (Leave empty to use the one from the odom topic)

- `odom_topic`: The name of the nav_msgs/Odometry topic to subscribe to.

- `inverse_tf`: If true, the tf will be generated from the base_link to the odom frame. Otherwise, by default, it will be generated from the odom frame to the base_link. (default: false)

- `use_original_timestamp`: Deprecated since its logic was inverted. Use the `use_original_odom_timestamp` parameter instead. The new parameter defaults to the same behaviour as this one. If set, the node throws an error to ensure that the proper logic is used.

- `use_original_odom_timestamp`: If true, the tf will be generated using the timestamp from the odom message. Otherwise, by default, it will be generated using `now()`. (default: true)
