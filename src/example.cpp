#include <iostream>
#include <fstream>
#include <array>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>

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

int main()
{
  kinlib::Manipulator baxter_manipulator;
  
  std::array<std::string, 7> joint_names{"S0", "S1", "E0", "E1", "W0", "W1", "W2"};
  
  Eigen::MatrixXd joint_axes = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_axes.csv", 3, 7);
  Eigen::MatrixXd joint_q = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_q.csv", 3, 7);
  Eigen::MatrixXd joint_limits = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_limits.csv", 7, 3);
  Eigen::MatrixXd gst_0 = readCSV(std::string(KINLIB_RESOURCES_DIR) + "baxter_gst0.csv", 4, 4);
  
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
  Eigen::MatrixXd rand_joint_angles = readCSV(std::string(KINLIB_RESOURCES_DIR) + "joint_angles.csv", 485, 7);
  Eigen::MatrixXd matlab_fk_results = readCSV(std::string(KINLIB_RESOURCES_DIR) + "matlab_fk_results.csv", 1940, 4);
  
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
  kinlib::ErrorCodes plan_res = kin_solver.getMotionPlan(init_jnt_val, init_ee_g, goal_ee_g, motion_plan_result);
  
  if(plan_res == kinlib::ErrorCodes::OPERATION_SUCCESS)
  {
    std::cout << "[SUCCESS] Motion plan computed successfully!\n";
    std::cout << "[SUCCESS] Motion plan length : " << motion_plan_result.size() << std::endl;
  }
  else
  {
    std::cout << "[ ERROR ] Motion plan computation failed!\n";
  }
  
}
