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

int main() {
    Eigen::IOFormat CleanFmt(Eigen::FullPrecision,0,"\t","\n");
    kinlib::Manipulator panda_manipulator;

    std::array<std::string, 7> joint_names{"pandaJoint1", "pandaJoint2", "pandaJoint3", "pandaJoint4", "pandaJoint5", "pandaJoint6", "pandaJoint7"};

    Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_axes.csv");
    Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_q.csv");
    Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_joint_limit.csv");
    Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "panda_gst0.csv");
    std::vector<double> gripper_condition = readGripperCondition(std::string(KINLIB_RESOURCES_DIR) +"robot_state_Y2025M01D16_T161155.csv");
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
    nullSpace::Robot panda = nullSpace::getPandaRobot();

    kinlib::KinematicsSolver kin_solver(panda_manipulator);
    // std::vector<Eigen::VectorXd> all_motion_plans;
    Eigen::VectorXd init_jnt_val(7); 
    // init_jnt_val << -1.8, -1.2, 1.5, -2.0, 1.5, 1.0, 2.0;
    init_jnt_val  << 0., -0.7854, 0.,-2.3562, 0., 1.5708, 0.7854;
    Eigen::Matrix4d init_ee_g;
    kin_solver.getFK(init_jnt_val, init_ee_g);
    
    std::vector<Eigen::VectorXd> null_space_range = nullSpace::exploreNullSpaceRange(panda, init_jnt_val);

    Eigen::VectorXd goal_jnt_val(7); 
    goal_jnt_val << 0.9407,  -1.324,  -0.313,  -2.439,   0.02,    2.08, 0.02683;
    Eigen::Matrix4d goal_ee_g;
    kin_solver.getFK(goal_jnt_val, goal_ee_g);

    Eigen::VectorXd null_space_low_limit = null_space_range[150];
    Eigen::Matrix4d nullspace_low_ee_g;
    kin_solver.getFK(null_space_low_limit, nullspace_low_ee_g);


    Eigen::VectorXd null_space_up_limit = null_space_range[650];
    Eigen::Matrix4d nullspace_up_ee_g;
    kin_solver.getFK(null_space_up_limit, nullspace_up_ee_g);

    // // No need to re-declare init_ee_g here; it's already declared above
    std::vector<Eigen::VectorXd> motion_plan_from_low;
    std::vector<Eigen::VectorXd> motion_plan_from_up;
    kinlib::MotionPlanResult plan_info;

    kinlib::ErrorCodes plan_res_low = kin_solver.getMotionPlan(
                null_space_low_limit,
                nullspace_low_ee_g,
                goal_ee_g,
                motion_plan_from_low,
                plan_info);

    kinlib::ErrorCodes plan_res_up = kin_solver.getMotionPlan(
                null_space_up_limit,
                nullspace_up_ee_g,
                goal_ee_g,
                motion_plan_from_up,
                plan_info);

    
    saveMotionPlanToCSV(motion_plan_from_low, "motion_plan_from_low_nsr.csv");
    saveMotionPlanToCSV(motion_plan_from_up, "motion_plan_from_up_nsr.csv");


    std::vector<Eigen::VectorXd> null_space_range_new = nullSpace::exploreNullSpaceRange(panda, motion_plan_from_up.back());

    saveMotionPlanToCSV(null_space_range_new, "null_space_range_new.csv");
};