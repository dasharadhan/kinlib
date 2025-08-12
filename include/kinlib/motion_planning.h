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
    const std::vector<double> &gripper_condition,
    std::vector<double> &guiding_pose_gripper_cond,
    const std::vector<unsigned int> &gripper_change_index,
    double alpha = 0.30,
    bool aplha_is_scale = false);

class UserGuidedMotionPlanner
{
  public:
    UserGuidedMotionPlanner() = default;
    ~UserGuidedMotionPlanner() = default;

    static ErrorCodes planMotionForNewTaskInstance(
        Demonstration &demo, TaskInstance &new_task_instance, std::vector<Eigen::Matrix4d> &ee_pose_seq);
};

}