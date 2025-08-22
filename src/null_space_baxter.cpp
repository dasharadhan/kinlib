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

    Eigen::VectorXd init_jnt_val(7); 
    init_jnt_val << -0.985, -0.735, -0.722, 0.554, 1.191, 1.091, -0.451;
    Eigen::Matrix4d init_ee_g;
    kin_solver.getFK(init_jnt_val, init_ee_g);
    Eigen::VectorXd goal_jnt_val(7); 
    goal_jnt_val <<1.218, 0.324, -0.450, 0.989, -1.224, 1.149, -0.902;
    Eigen::Matrix4d goal_ee_g;
    kin_solver.getFK(goal_jnt_val, goal_ee_g);

    std::vector<Eigen::VectorXd> motion_plan_result_withNSP;
    kinlib::MotionPlanResult plan_info_with_NSP;
    nullSpace::Robot baxter = nullSpace::getBaxterRobot();
    Eigen::VectorXd current_jnt_config(7); 
    current_jnt_config = init_jnt_val;
    double outer_threshold = 0.0;
    double inner_threshold = 0.2;


    // motion plan with null space motion
    kinlib::ErrorCodes plan_res_NSP = kin_solver.getMotionPlanWithNSP(
        baxter,
        current_jnt_config,
        init_ee_g,
        goal_ee_g,
        motion_plan_result_withNSP,
        plan_info_with_NSP,
        outer_threshold,
        inner_threshold);
    saveMotionPlanToCSV(motion_plan_result_withNSP, "baxter_joints_withNSP.csv");
} 