/*
 * Copyright (C) 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef ROS_SERVICE_HXX
# define ROS_SERVICE_HXX


inline
const std::string&
ROSBinding::ROSService::service_get() const
{
  return service_;
}


inline
ROSBinding::ROSService::ServiceTypeSocket::ServiceTypeSocket(ROSService* rs)
  : rs_(rs)
{}


inline
ROSBinding::ROSService::ServiceTypeSocket::~ServiceTypeSocket()
{}


#endif // ! ROS_SERVICE_HXX
