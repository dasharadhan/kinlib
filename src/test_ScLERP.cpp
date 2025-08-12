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
    panda_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i], jnt_axis, jnt_q, jnt_limits, gst_0); 
  }
  
  kinlib::KinematicsSolver kin_solver(panda_manipulator);

  Eigen::VectorXd init_jnt_val(7);
  init_jnt_val << 0., -0.7854, 0.,-2.3562, 0., 1.5708, 0.7854;
  Eigen::Matrix4d init_ee_g;
  kin_solver.getFK(init_jnt_val, init_ee_g);
  std::cout << init_ee_g;
}