#include <iostream>
#include <fstream>
#include <array>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>
#include "kinlib/nullSpace_kinematics.h"
#include "kinlib/robot_parameter.h"
template<typename M>
M loadCSV (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    
    return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

template <typename DemoT>
bool saveGuidingPoses(const DemoT& demo, const std::string& path)
{
    std::ofstream out(path);
    if (!out) return false;

    constexpr int precision = 8;      // tweak as needed
    Eigen::IOFormat fmt(/*precision=*/precision,
                        /*flags   =*/Eigen::DontAlignCols,
                        /*coeffSep=*/" ",
                        /*rowSep  =*/"\n");

    for (size_t i = 0; i < demo.guiding_poses.size(); ++i) {
        out << "# object " << i + 1 << '\n';
        for (size_t j = 0; j < demo.guiding_poses[i].size(); ++j) {
            out << demo.guiding_poses[i][j].format(fmt) << "\n\n";
        }
    }
    return true;
}
void saveMotionPlanToCSV(const std::vector<Eigen::VectorXd>& motion_plan, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    for (const auto& joint_values : motion_plan) {
        for (int j = 0; j < joint_values.size(); ++j) {
            file << joint_values[j];
            if (j < joint_values.size() - 1) {
                file << ",";
            }
        }
        file << std::endl;
    }
    

    file.close();
    std::cout << "Motion plan saved to " << filename << std::endl;
}


void saveGripperConditionToCSV(const std::vector<double>& guiding_pose_gripper_cond,
                         const std::string& filename) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // Write all elements on one line, separated by commas
    for (size_t i = 0; i < guiding_pose_gripper_cond.size(); ++i) {
        file << guiding_pose_gripper_cond[i];
        if (i < guiding_pose_gripper_cond.size() - 1) {
            file << ",";
        }
    }
    file << std::endl;

    file.close();
    std::cout << "Motion plan (gripper condition) saved to " << filename << std::endl;
}



std::vector<double> readGripperCondition(const std::string& filename) {
    std::vector<double> values; 
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return values;
    }
    
    std::string line;
  
    if (std::getline(file, line)) {

    }
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        int colIndex = 0;
        while (std::getline(ss, cell, ',')) {

            if (colIndex == 92) {
                try {
                    double value = std::stod(cell);  
                    values.push_back(value);
                } catch (const std::invalid_argument& e) {

                    std::cerr << " can transfer 93th column to number: " << cell << std::endl;
                } catch (const std::out_of_range& e) {

                    std::cerr << "number out of range: " << cell << std::endl;
                }
                break;
            }
            ++colIndex;
        }
    }
    
    file.close();
    return values;
}

int main()
{
//   define panda robot
  Eigen::IOFormat CleanFmt(Eigen::FullPrecision,0,"\t","\n");
  kinlib::Manipulator panda_manipulator;
  
  std::array<std::string, 7> joint_names{"pandaJoint1", "pandaJoint2", "pandaJoint3", "pandaJoint4", "pandaJoint5", "pandaJoint6", "pandaJoint7"};

  Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_axes.csv");
  Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_q.csv");
  Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_limit.csv");
  Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_gst0.csv");
  std::vector<double> gripper_condition = readGripperCondition(std::string(KINLIB_RESOURCES_DIR) +"robot_state_Y2025M01D16_T161155.csv");
//   std::cout << "Joint Axes :\n" << joint_axes << "\n\n";
//   std::cout << "Joint Q :\n" << joint_q << "\n\n";
//   std::cout << "Joint Limits :\n" << joint_limits << "\n\n";
//   std::cout << "gst0 :\n" << gst_0 << "\n\n";
  std::cout << "gst0 :\n" << gripper_condition.size() << "\n\n";
  for(int i = 0; i < 7; i++)
  {
    Eigen::Vector4d jnt_axis;
    jnt_axis.head<3>() = joint_axes.block<3,1>(0, i);
    jnt_axis(3) = 0;
    
    Eigen::Vector4d jnt_q;
    jnt_q.head<3>() = joint_q.block<3,1>(0, i);
    jnt_q(3) = 0;
    
    kinlib::JointLimits jnt_limits;
    jnt_limits.lower_limit_ = joint_limits(i, 0);
    jnt_limits.upper_limit_ = joint_limits(i, 1);
    
    // Ignore gst_0 being passed as joint tip for every joint as joint tip is
    // not used to do any computation
    panda_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i], jnt_axis, jnt_q, jnt_limits, gst_0); 
  }
  
  kinlib::KinematicsSolver kin_solver(panda_manipulator);
  

// User Guided Motion Planner
  Eigen::MatrixXd recorded_demo = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "/CellingTileData/VR_cellingtile_demo.csv");
  Eigen::MatrixXd object_poses = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "/CellingTileData/VR_cellingtile_object_pose.csv");

  std::vector<Eigen::Matrix4d> recorded_ee_traj;
  std::vector<Eigen::Matrix4d> obj_poses;

  for(int i = 0; i < recorded_demo.rows()/4; i++)
  {
    Eigen::Matrix4d g = recorded_demo.block<4,4>((i*4),0);
    recorded_ee_traj.push_back(g);
  }

  for(int i = 0; i < object_poses.rows()/4; i++)
  {
    Eigen::Matrix4d g = object_poses.block<4,4>((i*4),0);
    obj_poses.push_back(g);
  }
  
  std::cout << "ee_pos before filtered is :\n" << recorded_ee_traj.size() << std::endl;

  std::vector<Eigen::Matrix4d> filtered_ee_traj;
  std::vector<double> filtered_gripperCond;
  std::vector<unsigned int> gripper_change_index;
  kinlib::filterSE3Sequence(recorded_ee_traj,filtered_ee_traj, gripper_condition, filtered_gripperCond,gripper_change_index);
  
//   for(int i = 0; i < gripper_change_index.size(); i++)
//   {
//     std::cout << "\ngripper_change_index are " << gripper_change_index[i] << '\n';
//   }

  std::vector<double> guiding_pose_gripper_cond;
  std::cout << "ee_pos after filtered is :\n" << filtered_ee_traj.size() << std::endl;
  kinlib::Demonstration demo = kinlib::saveDemonstration(filtered_ee_traj,obj_poses,filtered_gripperCond,
                                                        guiding_pose_gripper_cond,gripper_change_index,0.2);
  
  // Guiding poses
  for(int i = 0; i < demo.guiding_poses.size(); i++)
  {
    std::cout << "\nGuiding poses associated with object " << i+1 << '\n';
    for(int j = 0; j < demo.guiding_poses[i].size(); j++)
    {
      std::cout << demo.guiding_poses[i][j] << '\n';
    }
  }


  // saveGuidingPoses(demo, "guiding_poses.txt");
  // Eigen::MatrixXd target_poses = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "/CellingTileData/VR_cellingtile_new_object_pose.csv");

  Eigen::MatrixXd pickup_poses = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "/CellingTileData/celling_tile_grasp_poses.csv");
  Eigen::MatrixXd loose_poses = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "/CellingTileData/celling_tile_loose_poses.csv");
  std::vector<Eigen::Matrix4d> target_obj_poses;

  std::vector<Eigen::Matrix4d> pick_up_obj_poses;
  std::vector<Eigen::Matrix4d> loose_obj_poses;


  for(int i = 0; i < pickup_poses.rows()/4; i++)
  {
    Eigen::Matrix4d g = pickup_poses.block<4,4>((i*4),0);
    pick_up_obj_poses.push_back(g);
  }

  for(int i = 0; i < loose_poses.rows()/4; i++)
  {
    Eigen::Matrix4d g = loose_poses.block<4,4>((i*4),0);
    loose_obj_poses.push_back(g);

  }

  std::vector<std::vector<Eigen::Matrix4d>> all_guiding_poses;

  for (int i = 0; i < (int)pick_up_obj_poses.size(); ++i) {
      kinlib::TaskInstance new_task_instance = demo.task_instance; 

      // general case
      // new_task_instance.object_poses[0] = loose_obj_poses[i];

      //  manually adjust coordinates 
      new_task_instance.object_poses[0](0,3) = loose_obj_poses[i](0,3); 
      new_task_instance.object_poses[0](1,3) = loose_obj_poses[i](1,3);
      new_task_instance.object_poses[0](2,3) = loose_obj_poses[i](2,3);

      std::vector<Eigen::Matrix4d> new_motion_plan;
      kinlib::ErrorCodes res = kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demo, new_task_instance, new_motion_plan);
      std::vector<Eigen::Matrix4d> merged;
      merged.reserve(1 + new_motion_plan.size());

      merged.push_back(pick_up_obj_poses[i]);

      merged.insert(merged.end(), new_motion_plan.begin(), new_motion_plan.end());
      all_guiding_poses.push_back(std::move(merged));
  }

  std::vector<Eigen::VectorXd> all_motion_plans;
  std::vector<double> motion_plans_gripper_cond;
  guiding_pose_gripper_cond.insert(guiding_pose_gripper_cond.begin(), 0.);


  nullSpace::Robot panda = nullSpace::getPandaRobot();

  double outer_threshold = 0.01;
  double inner_threshold = 0.3;
  Eigen::VectorXd init_jnt_val(7);
  init_jnt_val  << 0., -0.7854, 0.,-2.3562, 0., 1.5708, 0.7854;
//   init_jnt_val << 0.9407,  -1.324,  -0.313,  -2.439,   0.02,    2.08, 0.02683;
  Eigen::Matrix4d init_ee_g;
  kin_solver.getFK(init_jnt_val, init_ee_g);

  std::cout << all_guiding_poses[0].size();
  for (int i = 0; i < all_guiding_poses.size(); i++)
  {
    for (int j = 0; j < all_guiding_poses[0].size(); j++)
    {
      Eigen::Matrix4d goal_ee_g = all_guiding_poses[i][j];

      // No need to re-declare init_ee_g here; it's already declared above
      std::vector<Eigen::VectorXd> motion_plan_result;
      
      kinlib::MotionPlanResult plan_info;

      kinlib::ErrorCodes plan_res = kin_solver.getMotionPlanWithNSP(panda,init_jnt_val, init_ee_g, goal_ee_g, motion_plan_result, plan_info,outer_threshold,
        inner_threshold);

      // hard code the gripper condition since the VR data does not have the message

      if (j >= 1 && j <= 14){
        guiding_pose_gripper_cond[j] = 1.;
      }
      for (int k = 0; k < motion_plan_result.size(); k++) {
          motion_plans_gripper_cond.push_back(guiding_pose_gripper_cond[j]);
      }

      all_motion_plans.insert(all_motion_plans.end(), motion_plan_result.begin(), motion_plan_result.end());
      init_jnt_val = motion_plan_result.back();
      saveMotionPlanToCSV(all_motion_plans, "VR_celling_tile_joints.csv");
      
    }
  
    // std::vector<Eigen::Matrix4d> new_motion_plan;
    // kinlib::ErrorCodes res = kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demo,new_task_instance,new_motion_plan);
  } 
  saveGripperConditionToCSV(motion_plans_gripper_cond, "celling_tile_gripper_condition.csv");
  
}
