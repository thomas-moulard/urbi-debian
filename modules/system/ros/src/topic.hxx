/*
 * Copyright (C) 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef ROS_TOPIC_HXX
# define ROS_TOPIC_HXX

inline
void
ROSBinding::ROSTopic::msg_callback(UValuePtr msg)
{
  libport::BlockLock l(msg_lock_);
  ev_msg.emit(*msg);
}


inline
const std::string&
ROSBinding::ROSTopic::topic_get() const
{
  return topic_;
}


inline
const ROSBinding::ROSTopic::TopicMsgPtr
ROSBinding::ROSTopic::topic_msg_get() const
{
  return topic_msg_;
}


inline
size_t
ROSBinding::ROSTopic::queue_size_get() const
{
  return queue_size_;
}

inline
void
ROSBinding::ROSTopic::queue_size_set(size_t size)
{
  queue_size_ = size;
}


inline
void
ROSBinding::ROSTopic::subscriber_connect(
    const ros::SingleSubscriberPublisher& sub)
{
  sub_connect.emit(sub.getSubscriberName());
}


inline
void
ROSBinding::ROSTopic::subscriber_disconnect(
    const ros::SingleSubscriberPublisher& sub)
{
  sub_disconnect.emit(sub.getSubscriberName());
}

#endif // ! ROS_TOPIC_HXX
