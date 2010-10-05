#include "ros/ros.h"
#include "uservices/addTwoInts.h"

static int nb_services = 0;

bool add(uservices::addTwoInts::Request  &req,
         uservices::addTwoInts::Response &res )
{
  res.s = req.a + req.b;
  ++nb_services;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "addTwoInts");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/utests/addTwoInts", add);
  while (nb_services < 2)
  {
    ros::spinOnce();
    sleep(1);
  }

  // Let time to serve.
  sleep(1);

  return 0;
}

