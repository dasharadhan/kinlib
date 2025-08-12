
#include <vector>
#include <utility>            // for std::pair
#include <Eigen/Dense>
#include <Eigen/Geometry>
#pragma once

namespace nullSpace{
    // —— Structs ——
    struct Robot {
        Eigen::Matrix4d                gst0;
        std::vector<Eigen::Vector3d>   axis_joints;
        std::vector<Eigen::Vector3d>   q_joints;
        std::vector<int>               type_joints;  // 0 = revolute, 1 = prismatic
        Eigen::Vector3d                S_0;
        Eigen::Vector3d                E_0;
        Eigen::Vector3d                W_0;
        Eigen::Vector3d                e_r;
        int                            S_idx;
        int                            E_idx;
        int                            W_idx;
        Eigen::Matrix<double,7,2>      joint_limits;
    };

    struct SewParams {
        Eigen::Vector3d p_S;
        Eigen::Vector3d p_E;
        Eigen::Vector3d p_W;
        Eigen::Vector3d p_SW;
        Eigen::Vector3d e_SW;
        Eigen::Vector3d p_SE;
        Eigen::Vector3d p_CE;
        Eigen::Vector3d e_CE;
        Eigen::Vector3d k_y;
        Eigen::Vector3d e_y;
        Eigen::Vector3d e_x;
        Eigen::Vector3d W_0;
    };

    // —— Utility functions ——
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::Matrix3d axisAngleToRot(const Eigen::Vector3d& axis, double theta);

    // —— Kinematics and Jacobians ——
    std::pair<Eigen::Matrix4d,std::vector<Eigen::Matrix4d>> manipDKin(const Robot& manipulator, const Eigen::VectorXd& theta);

    std::pair<double, SewParams> getSEWParams(const Robot& robot, const Eigen::VectorXd& theta);

    Eigen::Matrix<double,6,6> adjointOfG(const Eigen::Matrix4d& g);

    Eigen::MatrixXd spatialManipJac(const Robot& robot, const Eigen::VectorXd& theta);

    Eigen::MatrixXd pinv(const Eigen::MatrixXd& A, double tol = -1);

    Eigen::MatrixXd getAugmentedJacobian(const Robot& robot, const Eigen::VectorXd& theta);

    std::pair<bool,int> checkIfWithinJointLimits(const Robot& robot, const Eigen::VectorXd& joint_config);

    std::tuple<bool,int,bool> checkIfWithinSoftJointLimits(const Robot& robot, const Eigen::VectorXd& joint_config, const double& threshold);

    bool checkIfBacktoSoftJointLimits(
    const Robot&           robot,
    const Eigen::VectorXd& q,
    const double&          innerThreshold,
    const int&             robot_idx);
    
    std::vector<Eigen::VectorXd> exploreNullSpaceRange(
    const Robot& robot,     
    const Eigen::VectorXd& theta);  
    
    std::tuple<int,int,int> checkStepLimits( const Robot& robot, 
                                    Eigen::VectorXd theta, 
                                    bool& reach_up_limit,  
                                    const int& joint_idx,  
                                    const double& outer_threshold , 
                                    const double& inner_threshold );
    int decideSEWDirection(const Robot& robot, Eigen::VectorXd& theta, const bool& reach_up_limit, const int& joint_idx);
}
