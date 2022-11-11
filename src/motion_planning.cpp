#include <iostream>
#include "kinlib/motion_planning.h"

namespace kinlib
{

Demonstration saveDemonstration(
    std::vector<Eigen::Matrix4d> &ee_trajectory,
    std::vector<Eigen::Matrix4d> &obj_poses,
    double roi_radius)
{
  Demonstration demo;

  if(ee_trajectory.size() == 0 || obj_poses.size() == 0)
  {
    return demo;
  }

  demo.task_instance.object_poses = obj_poses;
  demo.recorded_ee_trajectory = ee_trajectory;

  std::vector<unsigned int> segs;

  getScrewSegments(ee_trajectory, segs, 0.015, 0.15);

  std::vector<Eigen::Matrix4d> sparse_ee_trajectory;
  sparse_ee_trajectory.push_back(ee_trajectory[0]);

  for(unsigned int seg_itr = 0; seg_itr < segs.size(); seg_itr++)
  {
    sparse_ee_trajectory.push_back(ee_trajectory[segs[seg_itr]]);
  }

  int traj_itr = 0;

  std::vector<unsigned int> obj_list;

  for(unsigned int obj_itr = 0; obj_itr < obj_poses.size(); obj_itr++)
  {
    obj_list.push_back(obj_itr);
  }

  bool nearest_obj_found = false;
  unsigned int nearest_obj_id = 0;

  while(traj_itr < sparse_ee_trajectory.size())
  {
    // Find the object that is nearest in position to the current end-effector pose
    for(int i = 0; i < obj_list.size(); i++)
    {
      double dist = positionDistance(sparse_ee_trajectory[traj_itr], obj_poses[obj_list[i]]);
      std::cout << "Distance from object " << obj_list[i] << " to end-effector pose " << traj_itr << " : " << dist << '\n';

      if(dist <= roi_radius)
      {
        nearest_obj_found = true;
        nearest_obj_id = obj_list[i];
        obj_list.erase(obj_list.begin() + i);
        break;
      }
    }

    if(nearest_obj_found)
    {
      std::vector<Eigen::Matrix4d> obj_guiding_poses;

      nearest_obj_found = false;

      std::cout << "Object " << nearest_obj_id << " close to end-effector pose " << traj_itr << '\n';
      // Find all the poses in the trajectory that lie inside the region of interest of the object
      while(traj_itr < sparse_ee_trajectory.size())
      {
        double dist_to_obj = positionDistance(sparse_ee_trajectory[traj_itr], obj_poses[nearest_obj_id]);
        std::cout << "Distance from object " << nearest_obj_id << " to end-effector pose " << traj_itr << " : " << dist_to_obj << '\n';
        if(dist_to_obj <= roi_radius)
        {
          // Compute the pose of the end-effector relative to the object
          Eigen::Matrix4d ee_pose_rel_to_obj = 
              getTransformationInv(obj_poses[nearest_obj_id]) * sparse_ee_trajectory[traj_itr];
          obj_guiding_poses.push_back(ee_pose_rel_to_obj);
          traj_itr++;
        }
        else
        {
          std::cout << "Object " << nearest_obj_id << " far from end-effector pose " << traj_itr << '\n';
          demo.guiding_poses.push_back(obj_guiding_poses);
          break;
        }
      }
    }
    else
    {
      traj_itr++;
    }
  }

  return demo;
}

ErrorCodes UserGuidedMotionPlanner::planMotionForNewTaskInstance(
    Demonstration &demo,
    TaskInstance &new_task_instance,
    std::vector<Eigen::Matrix4d> &ee_pose_seq)
{
  if(demo.guiding_poses.size() != new_task_instance.object_poses.size())
  {
    return ErrorCodes::PARAMETER_ERROR;
  }

  for(unsigned int i = 0; i < demo.guiding_poses.size(); i++)
  {
    for(unsigned int j = 0; j < demo.guiding_poses[i].size(); j++)
    {
      Eigen::Matrix4d ee_pose = new_task_instance.object_poses[i] * demo.guiding_poses[i][j];
      ee_pose_seq.push_back(ee_pose);
    }
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

}
