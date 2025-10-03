#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/native_enum.h>
#include <iostream>
#include <fstream>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>
#include <kinlib/robot_parameter.h>
#include <kinlib/nullSpace_kinematics.h>

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
    py::tuple motion_plan_for_goal_pose(Eigen::VectorXd init_joint_angle, Eigen::Matrix4d goal_pose, std::string robot);
    Eigen::Matrix4d forward_kinematics(Eigen::VectorXd joint_angle, std::string robot);
    py::tuple poses_for_demonstration(std::vector<Eigen::Matrix4d> demonstration_poses, std::vector<double> gripper_states, std::vector<Eigen::Matrix4d> passive_object_poses);
    py::tuple null_space_motion_plan(Eigen:: VectorXd init_joint_angle, Eigen::Matrix4d goal_pose, std::string robot);

    Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, "\t", "\n");
    bool is_init = false;
    kinlib::Manipulator baxter_manipulator;
    kinlib::Manipulator kinova_manipulator;
    nullSpace::Robot baxter_nullspace = nullSpace::getBaxterRobot();
    nullSpace::Robot kinova_nullspace = nullSpace::getKinovaRobot();

    std::vector<Eigen::VectorXd> FAIL_STATE; // i.e. empty list

    // Baxter data
    std::array<std::string, 7> baxter_joint_names{"S0", "S1", "E0", "E1", "W0", "W1", "W2"};
    Eigen::MatrixXd baxter_joint_axes_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_axes.csv");
    Eigen::MatrixXd baxter_joint_q_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_q.csv");
    Eigen::MatrixXd baxter_joint_limits_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_joint_limits.csv");
    Eigen::MatrixXd baxter_gst_0_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "baxter_gst0.csv");
    Eigen::Vector4d baxter_jnt_axis;
    Eigen::Vector4d baxter_jnt_q;
    kinlib::JointLimits baxter_jnt_limits;

    // Kinova data
    std::array<std::string, 7> kinova_joint_names{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    Eigen::MatrixXd kinova_joint_axes_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "kinova_joint_axes.csv");
    Eigen::MatrixXd kinova_joint_q_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "kinova_joint_q.csv");
    Eigen::MatrixXd kinova_joint_limits_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "kinova_joint_limits.csv");
    Eigen::MatrixXd kinova_gst_0_csv = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "kinova_gst0.csv");
    Eigen::Vector4d kinova_jnt_axis;
    Eigen::Vector4d kinova_jnt_q;
    kinlib::JointLimits kinova_jnt_limits;

    // constants for kinlib
    double pos_threshold = 0.005;
    double rot_threshold = 0.01;

    double outer_threshold = 0.0;
    double inner_threshold = 0.2;

    void init() {
        for(int i = 0; i < 7; i++) {
            baxter_jnt_axis.head<3>() = baxter_joint_axes_csv.block<3,1>(0, i);
            baxter_jnt_axis(3) = 0;
            
            baxter_jnt_q.head<3>() = baxter_joint_q_csv.block<3,1>(0, i);
            baxter_jnt_q(3) = 0;

            baxter_jnt_limits.lower_limit_ = baxter_joint_limits_csv(i, 0);
            baxter_jnt_limits.upper_limit_ = baxter_joint_limits_csv(i, 1);

            // Ignore baxter_gst_0_csv being passed as joint tip for every joint as joint tip is
            // not used to do any computation
            baxter_manipulator.addJoint(kinlib::JointType::Revolute, baxter_joint_names[i], baxter_jnt_axis, baxter_jnt_q, baxter_jnt_limits, baxter_gst_0_csv); 
        }

        for(int i = 0; i < 7; i++) {
            kinova_jnt_axis.head<3>() = kinova_joint_axes_csv.block<3,1>(0, i);
            kinova_jnt_axis(3) = 0;
            
            kinova_jnt_q.head<3>() = kinova_joint_q_csv.block<3,1>(0, i);
            kinova_jnt_q(3) = 0;

            kinova_jnt_limits.lower_limit_ = kinova_joint_limits_csv(i, 0);
            kinova_jnt_limits.upper_limit_ = kinova_joint_limits_csv(i, 1);

            kinova_manipulator.addJoint(kinlib::JointType::Revolute, kinova_joint_names[i], kinova_jnt_axis, kinova_jnt_q, kinova_jnt_limits, kinova_gst_0_csv);
        }
    }

    kinlib::KinematicsSolver solverForRobot(const std::string &robot) {
        if (robot == "BAXTER") {
            return kinlib::KinematicsSolver(baxter_manipulator);
        }
        else if (robot == "KINOVA") {
            return kinlib::KinematicsSolver(kinova_manipulator);
        } else {
            throw std::invalid_argument("[solverForRobot] Unrecognized value: " + robot);
        }
    }

    nullSpace::Robot nullspaceForRobot(const std::string &robot) {
        if (robot == "BAXTER") {
            return python_bindings::baxter_nullspace;
        } else if (robot == "KINOVA") {
            return python_bindings::kinova_nullspace;
        } else {
            throw std::invalid_argument("[nullspaceForRobot] Unrecognized value: " + robot);
        }
    }
    
    // Get motion plan for goal pose
    py::tuple motion_plan_for_goal_pose(Eigen::VectorXd init_joint_angle, Eigen::Matrix4d goal_pose, std::string robot) {
        if (!is_init) {
            python_bindings::init();
            is_init = true;
        }
        kinlib::KinematicsSolver kin_solver = python_bindings::solverForRobot(robot);
        Eigen::Matrix4d init_ee_pose;
        kin_solver.getFK(init_joint_angle, init_ee_pose);
        std::vector<Eigen::VectorXd> pose_result;
        kinlib::MotionPlanResult plan_info;
        kinlib::ErrorCodes plan_res = kin_solver.getMotionPlan(init_joint_angle, init_ee_pose, goal_pose, pose_result, plan_info);

        if (plan_res == kinlib::ErrorCodes::OPERATION_SUCCESS) {
            std::cout << "[SUCCESS] Motion plan computed successfully!\n";
            std::cout << "[SUCCESS] Motion plan length : " << pose_result.size() << std::endl;
            return py::make_tuple(pose_result, true);
        } else {
            std::cout << "[ ERROR ] Motion plan computation failed!\n";
            std::cout << "[ ERROR ] Motion plan length : " << pose_result.size() << std::endl;
            return py::make_tuple(pose_result, false);
        }
    }

    Eigen::Matrix4d forward_kinematics(Eigen::VectorXd joint_angle, std::string robot) {
        if (!is_init) {
            python_bindings::init();
            is_init = true;
        }
        kinlib::KinematicsSolver kin_solver = python_bindings::solverForRobot(robot);
        Eigen::Matrix4d ee_pose;
        kin_solver.getFK(joint_angle, ee_pose);
        return ee_pose;
    }

    // Get poses for demonstration
    py::tuple poses_for_demonstration(std::vector<Eigen::Matrix4d> demonstration_poses, std::vector<double> gripper_states, std::vector<Eigen::Matrix4d> passive_object_poses) {
        if (!is_init) {
            python_bindings::init();
            is_init = true;
        }
        
        std::vector<Eigen::Matrix4d> filtered_demonstration_poses;
        std::vector<double> filtered_gripper_cond;
        std::vector<unsigned int> gripper_change_idx;
        kinlib::filterSE3Sequence(demonstration_poses, filtered_demonstration_poses, gripper_states, filtered_gripper_cond, gripper_change_idx, pos_threshold, rot_threshold);

        kinlib::Demonstration demo = kinlib::saveDemonstration(filtered_demonstration_poses, passive_object_poses, gripper_states, filtered_gripper_cond, gripper_change_idx);
        
        std::vector<Eigen::Matrix4d> pose_result;
        kinlib::ErrorCodes plan_res = kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demo, demo.task_instance, pose_result);

        if (plan_res == kinlib::ErrorCodes::OPERATION_SUCCESS) {
            std::cout << "[SUCCESS] Demonstration poses computed successfully!\n";
            std::cout << "[SUCCESS] Demonstration pose sequence length : " << pose_result.size() << std::endl;
            return py::make_tuple(pose_result, true);
        } else {
            std::cout << "[ ERROR ] Demonstration pose computation failed!\n";
            std::cout << "[ ERROR ] Demonstration pose sequence length : " << pose_result.size() << std::endl;
            return py::make_tuple(pose_result, false);
        }
    }

    // Get motion plan for goal pose
    py::tuple null_space_motion_plan(Eigen::VectorXd init_joint_angle, Eigen::Matrix4d goal_pose, std::string robot) {
        if (!is_init) {
            python_bindings::init();
            is_init = true;
        }
        kinlib::KinematicsSolver kin_solver = python_bindings::solverForRobot(robot);
        nullSpace::Robot nullspace_robot = python_bindings::nullspaceForRobot(robot);
        Eigen::Matrix4d init_ee_pose;
        kin_solver.getFK(init_joint_angle, init_ee_pose);
        std::vector<Eigen::VectorXd> result_angles;
        kinlib::MotionPlanResult plan_info;
        kinlib::ErrorCodes plan_res = kin_solver.getMotionPlanWithNSP(nullspace_robot, init_joint_angle, init_ee_pose, goal_pose, result_angles, plan_info, outer_threshold, inner_threshold);

        if (plan_res == kinlib::ErrorCodes::OPERATION_SUCCESS) {
            std::cout << "[SUCCESS] Motion plan computed successfully!\n";
            std::cout << "[SUCCESS] Motion plan length : " << result_angles.size() << std::endl;
            return py::make_tuple(result_angles, true);
        } else {
            std::cout << "[ ERROR ] Motion plan computation failed!\n";
            std::cout << "[ ERROR ] Motion plan length : " << result_angles.size() << std::endl;
            return py::make_tuple(result_angles, false);
        }
    }
}

PYBIND11_MODULE(kinlib_cpp, m) {
    m.def("motion_plan_for_goal_pose", &python_bindings::motion_plan_for_goal_pose, "Compute motion plan for goal pose given initial joint angles.");
    m.def("forward_kinematics", &python_bindings::forward_kinematics, "Compute end-effector pose for given joint angles.");
    m.def("poses_for_demonstration", &python_bindings::poses_for_demonstration, "Compute poses for demonstration given initial EE pose.");
    m.def("null_space_motion_plan", &python_bindings::null_space_motion_plan, "Compute motion plan for goal pose using null space exploration.");
}