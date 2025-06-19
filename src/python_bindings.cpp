#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <iostream>
#include <fstream>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>

namespace py = pybind11;

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

namespace python_bindings {
    py::tuple motion_plan_for_goal_pose(Eigen::VectorXd init_joint_angle, Eigen::Matrix4d goal_pose);
    Eigen::Matrix4d forward_kinematics(Eigen::VectorXd joint_angle);

    Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, "\t", "\n");
    kinlib::Manipulator baxter_manipulator;
    bool is_init = false;

    std::vector<Eigen::VectorXd> FAIL_STATE; // i.e. empty list

    // Baxter data
    std::array<std::string, 7> joint_names{"S0", "S1", "E0", "E1", "W0", "W1", "W2"};
    Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_axes.csv");
    Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_q.csv");
    Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_limits.csv");
    Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_gst0.csv");
    Eigen::Vector4d jnt_axis;
    Eigen::Vector4d jnt_q;
    kinlib::JointLimits jnt_limits;

    void init() {
        for(int i = 0; i < 7; i++) {
            
            jnt_axis.head<3>() = joint_axes.block<3,1>(0, i);
            jnt_axis(3) = 0;

            
            jnt_q.head<3>() = joint_q.block<3,1>(0, i);
            jnt_q(3) = 0;

            jnt_limits.lower_limit_ = joint_limits(i, 0);
            jnt_limits.upper_limit_ = joint_limits(i, 1);

            // Ignore gst_0 being passed as joint tip for every joint as joint tip is
            // not used to do any computation
            baxter_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i], jnt_axis, jnt_q, jnt_limits, gst_0); 
        }
    }
    
    // Get motion plan for goal pose
    py::tuple motion_plan_for_goal_pose(Eigen::VectorXd init_joint_angle, Eigen::Matrix4d goal_pose) {
        if (!is_init) {
            python_bindings::init();
            is_init = true;
        }
        kinlib::KinematicsSolver kin_solver(baxter_manipulator);
        Eigen::Matrix4d init_ee_pose;
        kin_solver.getFK(init_joint_angle, init_ee_pose);
        std::vector<Eigen::VectorXd> motion_plan_result;
        kinlib::MotionPlanResult plan_info;
        kinlib::ErrorCodes plan_res = kin_solver.getMotionPlan(init_joint_angle, init_ee_pose, goal_pose, motion_plan_result, plan_info);

        if (plan_res == kinlib::ErrorCodes::OPERATION_SUCCESS) {
            std::cout << "[SUCCESS] Motion plan computed successfully!\n";
            std::cout << "[SUCCESS] Motion plan length : " << motion_plan_result.size() << std::endl;
            return py::make_tuple(motion_plan_result, true);
        } else {
            std::cout << "[ ERROR ] Motion plan computation failed!\n";
            std::cout << "[ ERROR ] Motion plan length : " << motion_plan_result.size() << std::endl;
            return py::make_tuple(motion_plan_result, false);
        }
    }

    Eigen::Matrix4d forward_kinematics(Eigen::VectorXd joint_angle) {
        if (!is_init) {
            python_bindings::init();
            is_init = true;
        }
        kinlib::KinematicsSolver kin_solver(baxter_manipulator);
        Eigen::Matrix4d ee_pose;
        kin_solver.getFK(joint_angle, ee_pose);
        return ee_pose;
    }
}

PYBIND11_MODULE(kinlib_cpp, m) {
    m.def("motion_plan_for_goal_pose", &python_bindings::motion_plan_for_goal_pose, "Compute motion plan for goal pose given initial joint angles.");
    m.def("forward_kinematics", &python_bindings::forward_kinematics, "Compute end-effector pose for given joint angles.");
}