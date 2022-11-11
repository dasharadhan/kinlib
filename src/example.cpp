#include <iostream>
#include <fstream>
#include <array>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>

/*
Eigen::MatrixXd readCSV(std::string file, int rows, int cols)
{
  std::ifstream in(file);
  
  std::string line;

  int row = 0;
  int col = 0;

  Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

  if (in.is_open()) {

    while (std::getline(in, line)) {

      char *ptr = (char *) line.c_str();
      int len = line.length();

      col = 0;

      char *start = ptr;
      for (int i = 0; i < len; i++) {

        if (ptr[i] == ',') {
          res(row, col++) = atof(start);
          start = ptr + i + 1;
        }
      }
      res(row, col) = atof(start);

      row++;
    }

    in.close();
  }
  return res;
}
*/

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

int main()
{
  Eigen::IOFormat CleanFmt(Eigen::FullPrecision,0,"\t","\n");
  kinlib::Manipulator baxter_manipulator;
  
  std::array<std::string, 7> joint_names{"S0", "S1", "E0", "E1", "W0", "W1", "W2"};
  
  // Eigen::MatrixXd joint_axes = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_axes.csv", 3, 7);
  // Eigen::MatrixXd joint_q = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_q.csv", 3, 7);
  // Eigen::MatrixXd joint_limits = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_limits.csv", 7, 3);
  // Eigen::MatrixXd gst_0 = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_gst0.csv", 4, 4);

  Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_axes.csv");
  Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_q.csv");
  Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_limits.csv");
  Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_gst0.csv");
  
  std::cout << "Joint Axes :\n" << joint_axes << "\n\n";
  std::cout << "Joint Q :\n" << joint_q << "\n\n";
  std::cout << "Joint Limits :\n" << joint_limits << "\n\n";
  std::cout << "gst0 :\n" << gst_0 << "\n\n";
  
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
    baxter_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i], jnt_axis, jnt_q, jnt_limits, gst_0); 
  }
  
  kinlib::KinematicsSolver kin_solver(baxter_manipulator);
  
  // Compare FK results with MATLAB results for a set of random joint angles
  // Eigen::MatrixXd rand_joint_angles = readCSV(std::string(KINLIB_RESOURCES_DIR) + "joint_angles.csv", 485, 7);
  // Eigen::MatrixXd matlab_fk_results = readCSV(std::string(KINLIB_RESOURCES_DIR) + "matlab_fk_results.csv", 1940, 4);
  Eigen::MatrixXd rand_joint_angles = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "joint_angles.csv");
  Eigen::MatrixXd matlab_fk_results = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "matlab_fk_results.csv");

  std::vector<Eigen::Matrix4d> ee_traj;
  kin_solver.getEndEffectorTrajectory(rand_joint_angles, ee_traj);
  
  for(int i = 0; i < rand_joint_angles.rows(); i++)
  {
    Eigen::VectorXd joint_angles = rand_joint_angles.block<1,7>(i, 0).transpose();
    Eigen::Matrix4d ee_g;
    
    kin_solver.getFK(joint_angles, ee_g);
    
    bool g_check = true;
    
    for(int j = 0; j < 4; j++)
    {
      for(int k = 0; k < 4; k++)
      {
        if((matlab_fk_results((i*4)+j,k) - ee_g(j,k)) > 1.0e-5)
        {
          g_check = false;
          break;
        }

        if((matlab_fk_results((i*4)+j,k) - ee_traj[i](j,k)) > 1.0e-5)
        {
          g_check = false;
          break;
        }
      }

      if(!g_check)
      {
        break;
      }
    }
    
    if(!g_check)
    {
      std::cout << "[ ERROR ] Computed results not matching with MATLAB results!\n";
    }
    else
    {
      std::cout << "[SUCCESS] Test " << i+1 << " successful!\n";
    }
  }
  
  // Get motion plan using ScLERP Planner
  Eigen::VectorXd init_jnt_val(7);
  init_jnt_val << 0.828,-0.588,-0.370,1.837,1.511,-1.186,-1.913;
  
  Eigen::Matrix4d goal_ee_g;
  goal_ee_g <<  -0.111717, -0.079090,  0.990587, 0.492032,
                -0.993455, -0.014946, -0.113234, 0.596342,
                 0.023761, -0.996755, -0.076903, 0.192753,
                        0,         0,         0,        1;

  Eigen::Matrix4d init_ee_g;
  kin_solver.getFK(init_jnt_val, init_ee_g);
  
  std::vector<Eigen::VectorXd> motion_plan_result;
  kinlib::MotionPlanResult plan_info;
  kinlib::ErrorCodes plan_res = kin_solver.getMotionPlan(init_jnt_val, init_ee_g, goal_ee_g, motion_plan_result, plan_info);
  
  if(plan_res == kinlib::ErrorCodes::OPERATION_SUCCESS)
  {
    std::cout << "[SUCCESS] Motion plan computed successfully!\n";
    std::cout << "[SUCCESS] Motion plan length : " << motion_plan_result.size() << std::endl;
  }
  else
  {
    std::cout << "[ ERROR ] Motion plan computation failed!\n";
  }

  // User Guided Motion Planner
  Eigen::MatrixXd recorded_demo = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "Demonstrations/ScoopAndPour1/ee_trajectory.csv");
  Eigen::MatrixXd object_poses = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "Demonstrations/ScoopAndPour1/object_poses.csv");

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

  kinlib::Demonstration demo = kinlib::saveDemonstration(recorded_ee_traj,obj_poses);
  
  // Guiding poses
  for(int i = 0; i < demo.guiding_poses.size(); i++)
  {
    std::cout << "\nGuiding poses associated with object " << i+1 << '\n';
    for(int j = 0; j < demo.guiding_poses[i].size(); j++)
    {
      std::cout << demo.guiding_poses[i][j] << '\n';
    }
  }

  kinlib::TaskInstance new_task_instance;

  new_task_instance.object_poses = demo.task_instance.object_poses;
  new_task_instance.object_poses[1](0,3) = new_task_instance.object_poses[1](0,3) - 0.1;
  new_task_instance.object_poses[1](1,3) = new_task_instance.object_poses[1](1,3) + 0.05;
  new_task_instance.object_poses[1](2,3) = new_task_instance.object_poses[1](2,3) + 0.1;

  Eigen::MatrixXd matlab_new_motion_plan_mat = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "Demonstrations/ScoopAndPour1/new_motion_plan.csv");

  std::vector<Eigen::Matrix4d> matlab_new_motion_plan;

  for(int i = 0; i < matlab_new_motion_plan_mat.rows()/4; i++)
  {
    Eigen::Matrix4d temp_g = matlab_new_motion_plan_mat.block<4,4>(i*4,0);
    matlab_new_motion_plan.push_back(temp_g);
  }

  std::vector<Eigen::Matrix4d> new_motion_plan;
  kinlib::ErrorCodes res = kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demo,new_task_instance,new_motion_plan);

  std::cout << "\nAlgorithm 1 Verification :\n";
  if(new_motion_plan.size() == matlab_new_motion_plan.size())
  {
    for(int i = 0; i < new_motion_plan.size(); i++)
    {
      Eigen::Matrix4d diff = new_motion_plan[i] - matlab_new_motion_plan[i];
      for(int l = 0; l < 4; l++)
      {
        for(int k = 0; k < 4; k++)
        {
          if(abs(diff(l,k)) > 1e-5)
          {
            std::cout << "[ ERROR ] Motion plan computation failed!\n";
            break;
          }
        }
      }

      std::cout << "[SUCCESS] Pose " << i << " correct!\n";
    }
  }
  else
  {
    std::cout << "[ ERROR ] Motion plan computation failed!\n";
  }

  std::cout << "\nAlgorithm 1 Output :\n";
  for(auto g : new_motion_plan)
  {
    std::cout << g << '\n';
  }
}
