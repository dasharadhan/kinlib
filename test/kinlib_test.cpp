#include <ros/ros.h>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib_test/config.h>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <armadillo>
#include <fstream>

class KinlibTest : public testing::Test
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "knilib_test");

  ros::NodeHandle nh;

  ROS_INFO("Node Starting!");

  arma::
}
