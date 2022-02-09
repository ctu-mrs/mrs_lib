

  std::string from(const geometry_msgs::TransformStamped& msg)
  {
    return msg.header.frame_id;
  }

  std::string to(const geometry_msgs::TransformStamped& msg)
  {
    return msg.child_frame_id;
  }

  geometry_msgs::TransformStamped inverse(const geometry_msgs::TransformStamped& msg)
  {
    geometry_msgs::TransformStamped tf = transform_stamped_;
    tf2::Transform tf2;
    tf2::fromMsg(tf.transform, tf2);
    tf2 = tf2.inverse();
    tf.transform = tf2::toMsg(tf2);
    std::swap(tf.header.frame_id, tf.child_frame_id);
    return tf;
  }
