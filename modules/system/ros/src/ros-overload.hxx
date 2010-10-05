/*
 * Copyright (C) 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef ROS_OVERLOAD_HXX
# define ROS_OVERLOAD_HXX

inline
ros::NodeHandleExt::NodeHandleExt(const std::string& ns)
  : ros::NodeHandle(ns)
{}


inline
ros::SubscribeOptionsExt::SubscribeOptionsExt()
  : ros::SubscribeOptions()
{}

inline
ros::AdvertiseOptionsExt::AdvertiseOptionsExt()
  : ros::AdvertiseOptions()
{}


inline
ros::SubscriptionCallbackHelperExt::~SubscriptionCallbackHelperExt()
{}


inline
bool
ros::SubscriptionCallbackHelperExt::isConst()
{
  return fixed_size_;
}

#endif // ROS_OVERLOAD_HXX
