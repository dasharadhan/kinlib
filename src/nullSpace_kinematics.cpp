#include "kinlib/nullSpace_kinematics.h"
#include <iostream>
#include <cmath>               // for M_PI
#include <vector>
#include <utility>            // for std::pair
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>
#include <fstream>

namespace nullSpace{
// ——— Utility functions ———
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return m;
}

Eigen::Matrix3d axisAngleToRot(const Eigen::Vector3d& axis, double theta) {
    return Eigen::AngleAxisd(theta, axis.normalized()).toRotationMatrix();
}

// ——— manipDKin now correctly takes a PandaRobot by const‐ref ———
std::pair<Eigen::Matrix4d,std::vector<Eigen::Matrix4d>>
manipDKin(const Robot& manipulator, const Eigen::VectorXd& theta) {
    Eigen::Matrix4d g_base = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Matrix4d> transforms;
    transforms.reserve(manipulator.type_joints.size());

    for (size_t i = 0; i < manipulator.type_joints.size(); ++i) {
        Eigen::Matrix4d g_temp = Eigen::Matrix4d::Identity();
        if (manipulator.type_joints[i] == 0) {
            // Revolute
            Eigen::Matrix3d R = axisAngleToRot(manipulator.axis_joints[i], theta[i]);
            g_temp.block<3,3>(0,0) = R;
            g_temp.block<3,1>(0,3) =
                (Eigen::Matrix3d::Identity() - R) * manipulator.q_joints[i];
        } else {
            // Prismatic
            g_temp.block<3,1>(0,3) = theta[i] * manipulator.axis_joints[i];
        }
        g_base *= g_temp;
        transforms.push_back(g_base);
    }

    // append the fixed end-effector offset
    Eigen::Matrix4d g_base_tool = g_base * manipulator.gst0;
    return { g_base_tool, transforms };
}

std::pair<double, SewParams> getSEWParams(const Robot& robot, const Eigen::VectorXd& theta){
    SewParams sew;
    // auto [g_base_tool, transforms] = manipDKin(robot, theta);

    std::pair<Eigen::Matrix4d, std::vector<Eigen::Matrix4d>> result = manipDKin(robot, theta);

    Eigen::Matrix4d g_base_tool = result.first;
    std::vector<Eigen::Matrix4d> transforms = result.second;
    sew.p_S = robot.S_0;
   
    Eigen::Vector4d tmp1 = transforms[robot.E_idx - 1] * Eigen::Vector4d(robot.E_0.x(), robot.E_0.y(), robot.E_0.z(), 1.0);
    sew.p_E = tmp1.head<3>();

    Eigen::Vector4d tmp2 = transforms[robot.W_idx - 1] * Eigen::Vector4d(robot.W_0.x(), robot.W_0.y(), robot.W_0.z(), 1.0);
    sew.p_W = tmp2.head<3>();

    sew.p_SW = sew.p_W - sew.p_S;
    sew.e_SW = sew.p_SW.normalized();

    sew.p_SE = sew.p_E - sew.p_S;
    sew.p_CE = -skew(sew.e_SW) * skew(sew.e_SW) * sew.p_SE;
    sew.e_CE = sew.p_CE.normalized();

    sew.k_y = skew(sew.p_SW) * robot.e_r;
    sew.e_y = sew.k_y.normalized();
    sew.e_x = skew(sew.e_y) * sew.e_SW;
    double sew_angle = std::atan2(
        sew.e_SW.dot( skew(robot.e_r) * sew.p_CE ),
        robot.e_r.dot(sew.p_CE)
    );

    return { sew_angle, sew };

};

Eigen::Matrix<double,6,6> adjointOfG(const Eigen::Matrix4d& g) {
    Eigen::Matrix3d R = g.block<3,3>(0,0);
    Eigen::Vector3d p = g.block<3,1>(0,3);
    Eigen::Matrix<double,6,6> Ad = Eigen::Matrix<double,6,6>::Zero();
    Ad.block<3,3>(0,0) = R;     
    Ad.block<3,3>(3,3) = R;           
    Ad.block<3,3>(0,3) = skew(p) * R;  
    return Ad;
}

Eigen::MatrixXd spatialManipJac(const Robot& robot,
                                const Eigen::VectorXd& theta)
{
    int N = (int)robot.type_joints.size();

    // result = zeros(6, N);
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, N);

    // joint_twists = zeros(6, N);
    Eigen::MatrixXd joint_twists = Eigen::MatrixXd::Zero(6, N);

    // g = zeros(4,4,N);
    std::vector<Eigen::Matrix4d> g(N);
    for (int i = 0; i < N; ++i) {
        g[i] = Eigen::Matrix4d::Identity();
    }

    //  joint_twists
    for (int i = 0; i < N; ++i) {
        if (robot.type_joints[i] == 0) {
            // Revolute
            joint_twists.block<3,1>(0,i) = 
                -robot.axis_joints[i].cross(robot.q_joints[i]);
            joint_twists.block<3,1>(3,i) = robot.axis_joints[i];
        } else {
            // Prismatic
            joint_twists.block<3,1>(0,i) = robot.axis_joints[i];
        }
    }


    for (int i = 1; i < N; ++i) {
        for (int j = 0; j < i; ++j) {
            Eigen::Matrix4d temp_g = Eigen::Matrix4d::Identity();
            if (robot.type_joints[j] == 0) {
                // Revolute
                Eigen::Matrix3d R = axisAngleToRot(
                    robot.axis_joints[j], theta[j]
                );
                temp_g.block<3,3>(0,0) = R;
                temp_g.block<3,1>(0,3) = 
                    (Eigen::Matrix3d::Identity() - R) * robot.q_joints[j];
            } else {
                // Prismatic
                temp_g.block<3,1>(0,3) = 
                    theta[j] * robot.axis_joints[j];
            }
            g[i-1] = g[i-1] * temp_g;
        }
    }

    result.col(0) = joint_twists.col(0);

    // result[:,i] = Ad(g[:,:,i-1]) * joint_twists[:,i]
    for (int i = 1; i < N; ++i) {
        result.col(i) = adjointOfG(g[i-1]) * joint_twists.col(i);
    }

    return result;
}


Eigen::MatrixXd pinv(const Eigen::MatrixXd& A, double tol) {

    // Compute “thin” U and V:
    //   ComputeThinU  => U is m×r instead of m×m
    //   ComputeThinV  => V is n×r instead of n×n
    Eigen::JacobiSVD<Eigen::MatrixXd> svd (A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Singular values σ₁…σ_r
    const Eigen::VectorXd& S = svd.singularValues();

    int m = A.rows(), n = A.cols();
    if (tol < 0) {
        double eps = std::numeric_limits<double>::epsilon();
        tol = std::max(m, n) * eps * S.maxCoeff();
    }

    // Invert singular values above tol
    Eigen::VectorXd S_inv = S;
    for (int i = 0; i < S.size(); ++i) {
        S_inv(i) = (S(i) > tol ? 1.0 / S(i) : 0.0);
    }

    // A⁺ = V * Σ⁺ * Uᵀ
    return svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
}

Eigen::MatrixXd getAugmentedJacobian(const Robot& robot, const Eigen::VectorXd& theta){
    // auto [sew_angle, sew] = getSEWParams(robot, theta);
    std::pair<double, SewParams> result = getSEWParams(robot, theta);
    double sew_angle = result.first;
    SewParams sew   = result.second;
    Eigen::MatrixXd J_s = spatialManipJac(robot, theta);
    int n = J_s.cols();
    Eigen::RowVectorXd J_psi_E = 
        (skew(sew.e_SW) * sew.e_CE).transpose() / sew.p_CE.norm();

    double alpha = (1.0 / sew.k_y.norm()) * sew.e_SW.dot(robot.e_r);
    Eigen::RowVectorXd part1 = alpha * sew.e_y.transpose();
    double beta = (1.0 / (sew.p_SW.norm() * sew.p_CE.norm()))
               * sew.e_SW.dot(sew.p_SE);
    Eigen::RowVectorXd part2 = beta * (skew(sew.e_SW) * sew.e_CE).transpose();
    Eigen::RowVectorXd J_psi_W = part1 - part2;

    Eigen::Matrix<double,3,6> J_E_a;
    J_E_a << Eigen::Matrix3d::Identity(), -skew(sew.p_E);
    Eigen::MatrixXd J_E_s = Eigen::MatrixXd::Zero(6, n);
    J_E_s.leftCols(robot.E_idx) = J_s.leftCols(robot.E_idx);
    Eigen::MatrixXd J_E = J_E_a * J_E_s;  // 3×n
    Eigen::Matrix<double,3,6> J_W_a;
    J_W_a << Eigen::Matrix3d::Identity(), -skew(sew.p_W);
    Eigen::MatrixXd J_W = J_W_a * J_s;    // 3×n

    // 7) Combine J_ψ = J_ψ,E·J_E + J_ψ,W·J_W  → 1×n
    Eigen::RowVectorXd J_psi = J_psi_E * J_E + J_psi_W * J_W;

    // 8) Stack [J_s; J_ψ] → (6+1)×n
    Eigen::MatrixXd J_a(7, n);
    J_a.topRows(6)    = J_s;
    J_a.bottomRows(1) = J_psi;

    return J_a;
};


int decideSEWDirection(const Robot& robot, Eigen::VectorXd& theta, const bool& reach_up_limit, const int& joint_idx){

        Eigen::VectorXd q_next;  
        Eigen::MatrixXd J_a = nullSpace::getAugmentedJacobian(robot, theta);
        Eigen::MatrixXd J_pinv = nullSpace::pinv(J_a);
        Eigen::VectorXd err = Eigen::VectorXd::Zero(J_a.rows());
        err(err.size()-1) = 0.1;
        Eigen::VectorXd dq = J_pinv * err;
        q_next = theta + 0.05 * dq;
        Eigen::VectorXd delta_q = q_next - theta;
        if (reach_up_limit && delta_q(joint_idx) >0){
            return -1;
        }
        if (!reach_up_limit && delta_q(joint_idx) <0){
            return -1;
        }
        return 1;

}


std::tuple<int,int,int,bool> checkStepLimits( const Robot& robot, 
                                    Eigen::VectorXd theta, 
                                    bool& reach_up_limit,  
                                    const int& joint_idx,  
                                    const double& outer_threshold , 
                                    const double& inner_threshold,
                                    int max_steps_back_from_soft_joint_limit,
                                    int max_steps_around_soft_joint_limit){

    int SEW_direction = nullSpace::decideSEWDirection(robot, theta, reach_up_limit, joint_idx );

    int step_back_to_limit = 0;
    int step_out_of_limit = 0;
    Eigen::VectorXd q_next;  

    while(step_back_to_limit < max_steps_back_from_soft_joint_limit)
      {
        Eigen::MatrixXd J_a = nullSpace::getAugmentedJacobian(robot, theta);
        Eigen::MatrixXd J_pinv = nullSpace::pinv(J_a);
        Eigen::VectorXd err = Eigen::VectorXd::Zero(J_a.rows());
        err(err.size()-1) = 0.1 * SEW_direction;
        Eigen::VectorXd dq = J_pinv * err;
        q_next = theta + 0.05 * dq;
        bool back_to_range = nullSpace::checkIfBacktoSoftJointLimits(robot, q_next, inner_threshold, joint_idx);
        theta = q_next;
        ++step_back_to_limit;
        if(back_to_range)
            {
            std::cerr << "Joint " << joint_idx +1 << " take" << step_back_to_limit << "steps to move out from reaching soft joint limit\n";
            break;
            }

        // std::cout<<dq<<"\n";
      }

    while(step_out_of_limit < max_steps_around_soft_joint_limit)
      {
        Eigen::MatrixXd J_a = nullSpace::getAugmentedJacobian(robot, theta);
        Eigen::MatrixXd J_pinv = nullSpace::pinv(J_a);
        Eigen::VectorXd err = Eigen::VectorXd::Zero(J_a.rows());
        err(err.size()-1) = 0.1 * SEW_direction;
        Eigen::VectorXd dq = J_pinv * err;
        q_next = theta + 0.05 * dq;

        // auto [out_of_range, idx, _] = nullSpace::checkIfWithinSoftJointLimits(robot, q_next, outer_threshold);
        std::tuple<bool, int, bool> result =  nullSpace::checkIfWithinSoftJointLimits(robot, q_next, outer_threshold);

        bool out_of_range = std::get<0>(result);
        int idx = std::get<1>(result);

        ++step_out_of_limit;
        if(out_of_range)
        {
          std::cerr << "Joint " << idx+1 << " take" << step_out_of_limit << " steps to move out of soft joint limit\n";
          break;
        }
        theta = q_next;
        
      }
    bool success = step_back_to_limit < max_steps_back_from_soft_joint_limit && step_out_of_limit < max_steps_around_soft_joint_limit;
    return {SEW_direction, step_back_to_limit, step_out_of_limit, success};
};




std::pair<bool,int> checkIfWithinJointLimits(const Robot& robot,
                         const Eigen::VectorXd& joint_config)
{
    int n = robot.type_joints.size();
    bool out_of_range = false;
    int joint_idx = -1;  // -1 indicates “all OK”

    for (int i = 0; i < n; ++i) {
        double q = joint_config(i);
        double lo = robot.joint_limits(i, 0);
        double hi = robot.joint_limits(i, 1);
        if (q < lo || q > hi) {
            out_of_range = true;
            joint_idx = i;
            break;
        }
    }

    return { out_of_range, joint_idx };
}


std::tuple<bool,int,bool> checkIfWithinSoftJointLimits(const Robot& robot,
                         const Eigen::VectorXd& joint_config, const double& threshold)
{
    int n = static_cast<int>(robot.type_joints.size());
    bool limit_reached = false;
    int  joint_idx     = -1;  
    bool reached_up    = false;

    for (int i = 0; i < n; ++i) {
        double q  = joint_config(i);
        double lo = robot.joint_limits(i, 0) + threshold;
        double hi = robot.joint_limits(i, 1) - threshold;
        if (q < lo) {
            limit_reached = true;
            joint_idx     = i;
            reached_up    = false;
            break;
        }
        if (q > hi) {
            limit_reached = true;
            joint_idx     = i;
            reached_up    = true;
            break;
        }
    }

    return { limit_reached, joint_idx, reached_up };
}




bool checkIfBacktoSoftJointLimits(
    const Robot&           robot,
    const Eigen::VectorXd& q,
    const double&          innerThreshold,
    const int&             robot_idx
) {

    double qi = q(robot_idx);
    double lo = robot.joint_limits(robot_idx, 0) + innerThreshold;
    double hi = robot.joint_limits(robot_idx, 1) - innerThreshold;
    // if any joint is outside [lo,hi], fail
    if (qi < lo || qi > hi) {
        return false;
    }
    // every joint was inside
    return true;
}


// rewrite it to C++ 11 version
// std::vector<Eigen::VectorXd> exploreNullSpaceRange(
//     const Robot& robot,
//     const Eigen::VectorXd& theta0)
// {
//     double step_scale = 0.05;

//     auto withinLimits = [&](const Eigen::VectorXd& q)->bool {
//         for(int i = 0; i < q.size(); ++i) {
//             if (q(i) < robot.joint_limits(i,0)
//              || q(i) > robot.joint_limits(i,1))
//                 return false;
//         }
//         return true;
//     };

//     // negative direction  
//     std::vector<Eigen::VectorXd> negative_dir;
//     {
//         Eigen::VectorXd q = theta0;
//         while (true) {
//             Eigen::MatrixXd J_a    = nullSpace::getAugmentedJacobian(robot, q);
//             Eigen::MatrixXd J_pinv = nullSpace::pinv(J_a);
//             Eigen::VectorXd err    = Eigen::VectorXd::Zero(J_a.rows());
//             err(err.size()-1)      = 0.1;
//             Eigen::VectorXd dq     = J_pinv * err;

//             Eigen::VectorXd q_next = q - step_scale * dq;
//             if (!withinLimits(q_next)) break;
//             negative_dir.push_back(q_next);
//             q = q_next;
//         }
//     }

//     // positive direction
//     std::vector<Eigen::VectorXd> positive_dir;
//     {
//         Eigen::VectorXd q = theta0;
//         while (true) {
//             Eigen::MatrixXd J_a    = nullSpace::getAugmentedJacobian(robot, q);
//             Eigen::MatrixXd J_pinv = nullSpace::pinv(J_a);
//             Eigen::VectorXd err    = Eigen::VectorXd::Zero(J_a.rows());
//             err(err.size()-1)      = 0.1;
//             Eigen::VectorXd dq     = J_pinv * err;

//             Eigen::VectorXd q_next = q + step_scale * dq;
//             if (!withinLimits(q_next)) break;
//             positive_dir.push_back(q_next);
//             q = q_next;
//         }
//     }
//     std::vector<Eigen::VectorXd> all_configs;
//     for (auto it = negative_dir.rbegin(); it != negative_dir.rend(); ++it) {
//         all_configs.push_back(*it);
//     }
//     all_configs.push_back(theta0);
//     for (auto& q : positive_dir) {
//         all_configs.push_back(q);
//     }
//     return all_configs;
// }
}
