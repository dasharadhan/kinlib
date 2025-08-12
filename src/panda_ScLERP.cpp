#include <iostream>
#include <fstream>
#include <array>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

void saveMotionPlanToCSV(const std::string& filename, const std::vector<Eigen::VectorXd>& motion_plan_result) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file for writing: " + filename);
    }

    // Iterate through the motion plan result
    for (const auto& step : motion_plan_result) {
        for (int i = 0; i < step.size(); ++i) {
            file << step[i];  // Write each element
            if (i < step.size() - 1) {
                file << ",";  // Add a comma between elements
            }
        }
        file << "\n";  // Newline at the end of each row
    }

    file.close();
    std::cout << "[SUCCESS] Motion plan saved to: " << filename << std::endl;
}
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
    if (rows == 0 || values.size() % rows != 0) {
    throw std::runtime_error("Invalid CSV format: No rows or mismatched values.");
    }
    return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

int main()
{
  Eigen::IOFormat CleanFmt(Eigen::FullPrecision,0,"\t","\n");
  kinlib::Manipulator panda_manipulator;
  
  std::array<std::string, 7> joint_names{"pandaJoint1", "pandaJoint2", "pandaJoint3", "pandaJoint4", "pandaJoint5", "pandaJoint6", "pandaJoint7"};

  Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_axes.csv");
  Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_q.csv");
  Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_limit.csv");
  Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_gst0.csv");
  
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
    panda_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i], jnt_axis, jnt_q, jnt_limits, gst_0); 
  }
  
  kinlib::KinematicsSolver kin_solver(panda_manipulator);
  
  // Compare FK results with MATLAB results for a set of random joint angles
  // Eigen::MatrixXd rand_joint_angles = readCSV(std::string(KINLIB_RESOURCES_DIR) + "joint_angles.csv", 485, 7);
  // Eigen::MatrixXd matlab_fk_results = readCSV(std::string(KINLIB_RESOURCES_DIR) + "matlab_fk_results.csv", 1940, 4);
  // Eigen::MatrixXd rand_joint_angles = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "joint_angles.csv");
  // Eigen::MatrixXd matlab_fk_results = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "matlab_fk_results.csv");


  // Get motion plan using ScLERP Planner
  Eigen::VectorXd init_jnt_val(7);
  Eigen::VectorXd final_joints_val(7);
  init_jnt_val << 0.275417 ,  0.0694578, -0.140783 , -1.75514  ,  0.0852768, 1.70846  ,  0.967221;
//   final_joints_val << 0.1, 0.3, 0., -2., 0.5, 2., 0.6;
  // initial_joints = np.array([0, 0.3, 0, -2., 0, 3., 0.8])
  // goal_joints = [0.1, 0.3, 0, -2., 0.5, 2., 0.6]

  
//   Eigen::Matrix4d goal_ee_g;
//   kin_solver.getFK(final_joints_val, goal_ee_g);



// //  not work
//   Eigen::Matrix4d goal_ee_g;
//   goal_ee_g <<    -0.377347, -0.926067, -0.00307878, 0.145094,
//                   -0.92605, 0.377359, -0.00561095, 0.0941568,
//                   0.00635792, 0.000733828, -0.99998, 0.115169,
//                   0, 0, 0, 1;


// 0.990097 -0.0671011   -0.12331   0.565111
//  -0.060226   -0.99646   0.058665  0.0865523
//   -0.12681 -0.0506576  -0.990633   0.510112
//          0          0          0          1
 Eigen::Matrix4d goal_ee_g;
 goal_ee_g << 0.998726,   0.0503129, -0.00394707,    0.704536,
              0.050187,   -0.998367,  -0.0272825, -0.00328761,
              -0.00531329,   0.0270497,    -0.99962,    0.269314,
                0,           0,           0,           1;


  Eigen::Matrix4d init_ee_g;
  kin_solver.getFK(init_jnt_val, init_ee_g);
  
  std::cout << " init_ee_g is !!!!!!!!!!!";
  std::cout << init_ee_g;

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

  std::cout << "Motion Plan Result:" << std::endl;
    for (const auto& vector : motion_plan_result) {
        std::cout << vector.transpose() << std::endl; // Print each vector as a row
    }
  try {
        saveMotionPlanToCSV("motion_plan_result_(0.1-0.2).csv", motion_plan_result);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
  
}
