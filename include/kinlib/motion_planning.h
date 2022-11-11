#pragma once

#include "kinlib_kinematics.h"

namespace kinlib
{

struct TaskInstance
{
  std::vector<Eigen::Matrix4d> object_poses;
};

struct Demonstration
{
  TaskInstance task_instance;
  std::vector<Eigen::Matrix4d> recorded_ee_trajectory;
  std::vector< std::vector<Eigen::Matrix4d> > guiding_poses;
};

Demonstration saveDemonstration(
    std::vector<Eigen::Matrix4d> &ee_trajectory,
    std::vector<Eigen::Matrix4d> &obj_poses,
    double roi_radius = 0.30);

class UserGuidedMotionPlanner
{
  public:
    UserGuidedMotionPlanner() = default;
    ~UserGuidedMotionPlanner() = default;

    static ErrorCodes planMotionForNewTaskInstance(
        Demonstration &demo, TaskInstance &new_task_instance, std::vector<Eigen::Matrix4d> &ee_pose_seq);
};

}