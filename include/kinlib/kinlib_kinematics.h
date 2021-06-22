/*
 * Kinematic Solvers for kinlib
 */

/* Author: Dasharadhan Mahalingam */

#pragma once

#include <vector>
#include <string>
#include <map>
#include <iterator>

#include <Eigen/Dense>
#include <Eigen/Geometry>

//#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "kinlib.h"
#include "manipulator.h"
#include "DualQuat.h"

namespace kinlib
{
/*!
  \brief    To get distance between two rigid transformations

  \details  Returns the Eucledian distance between two given rigid body
            transformations

  \param    t1    Rigid body transformation
  \param    t2    Rigid body transformation

  \return   Eucledian distance
*/
double positionDistance(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2);

/*!
  \brief    To get distance between two points in space

  \details  Returns the Eucledian distance between two given points in space

  \param    p1    Point in space
  \param    p2    Point in space

  \return   Eucledian distance
*/
double positionDistance(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2);

/*!
  \brief    To get distance in rotation between two rigid transformations

  \details  Returns the distance in rotation between two given rigid body
            transformations

  \param    t1    Rigid body transformation
  \param    t2    Rigid body transformation

  \return   Distance in rotation
*/
double rotationDistance(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2);

/*!
  \brief    To get distance in rotation between two quaternions

  \details  Returns the distance in rotation between two quaternions which
            represent rigid body rotation

  \param    q1    Quaternion representing rotation
  \param    q2    Quaternion representing rotation

  \return   Distance in rotation
*/
double rotationDistance(const Eigen::Quaterniond &q1, 
                        const Eigen::Quaterniond &q2);

/*!
  \brief    To get distance in rotation between two dual quaternions

  \details  Returns the distance in rotation between two dual quaternions which
            represent rigid body rotation

  \param    dq_1    Dual Quaternion representing rigid transformation
  \param    dq_2    Dual Quaternion representing rigid transformation

  \return   Distance in rotation
*/
double rotationDistance(eigen_ext::DualQuat &dq_1, 
                        eigen_ext::DualQuat &dq_2);

/*!
  \brief    Get skew matrix for given vector

  \details  Returns the skew matrix for the give vector

  \param    vec   Vector

  \returns  Skew-matrix of vector %vec
*/
Eigen::Matrix3d getSkewMatrix(const Eigen::VectorXd &vec);

/*!
  \brief    Get inverse of a rigid body transformation

  \details  Returns the inverse for the given rigid body transformation

  \param    g     Rigid body transformation

  \returns  Inverse of %g
*/
Eigen::Matrix4d getTransformationInv(const Eigen::Matrix4d &g);

/*!
  \brief    Get adjoint of a transformation

  \details  Returns the adjoint of the the given rigid body transformation

  \param    g     Rigid body transformation

  \returns  Adjoint of rigid body transformation %g
*/
Eigen::Matrix<double,6,6> getAdjoint(const Eigen::Matrix4d &g);

class KinematicsSolver
{
  public:
    /*!
      \brief    Constructor to initialize class object
    */
    KinematicsSolver();

    /*!
      \brief    Constructor to initialize class object
    */
    KinematicsSolver(Manipulator manip);

    /*!
      \brief    To get Manipulator object of the kinematic solver
    */
    Manipulator getManipulator(void);

    /*!
      \brief    Forward Kinematics of manipulator

      \details  This function solves the forward kinematics of the manipulator
                for the given joint values and returns the end-effector pose

      \param    jnt_values    Joint values of manipulator
      \param    g_base_tool   Variable to store FK Results

      \return   Operation status
    */
    ErrorCodes getFK( const Eigen::VectorXd &jnt_values, 
                      Eigen::Matrix4d &g_base_tool);

    /*!
      \brief    Get spatial jacobian of manipulator

      \details  This function solves for the spatial jacobian of the manipulator
                for the given joint values

      \param    jnt_values    Joint values of manipulator
      \param    manip_jac     Variable to store Jacobian of manipulator

      \return   Operation status
    */
    ErrorCodes getSpatialJacobian(const Eigen::VectorXd &jnt_values,
                                  Eigen::MatrixXd &manip_jac);

    /*!
      \brief    Resolved Motion Rate Control step determination

      \details  Determine joint angle change based on RMRC to achieve goal
                configuration

      \param    dq_i                    Initial end-effector configuration
      \param    dq_f                    Goal end-effector configuration
      \param    jnt_values              Current joint values of manipulator
      \param    jnt_values_increment    Increment in joint angles for RMRC

      \return   Operation status
    */
    ErrorCodes getResolvedMotionRateControlStep(
        eigen_ext::DualQuat &dq_i,
        eigen_ext::DualQuat &dq_f,
        const Eigen::VectorXd &jnt_values,
        Eigen::VectorXd &jnt_values_increment);

    /*!
      \brief    Get motion plan for manipulator

      \details  This function solves for a motion plan based on the given
                initial joint angles and final end-effector configuration using
                Screw Linear Interpolation 

      \param    init_jnt_values     Initial joint values of manipulator
      \param    g_i                 Initial end-effector configuration
      \param    g_f                 Final end-effector configuration required
      \param    jnt_trajectory      Variable to store motion plan

      \return   Operation status
    */
    ErrorCodes getMotionPlan( const Eigen::VectorXd &init_jnt_values,
                              const Eigen::Matrix4d &g_i,
                              const Eigen::Matrix4d &g_f,
                              trajectory_msgs::JointTrajectory &jnt_trajectory);
                              //std::vector<Eigen::VectorXd> &jnt_angle_seq);

  private:
    /*!
      \brief    Manipulator description for solver

      \details  Stores the properties for the manipulator for which we are
                solving the kinematics for
    */
    Manipulator manipulator_;
};
}
