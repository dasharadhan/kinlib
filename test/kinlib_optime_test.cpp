#include <kinlib/kinlib_kinematics.h>
#include <kinlib_test/config.h>
#include <boost/filesystem/path.hpp>
#include <iostream>
#include <chrono>
#include <random>

int main(int argc, char** argv)
{
  std::mt19937 generator(1729);
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  bool op_result;
  boost::filesystem::path res_dir(KINLIB_TEST_RESOURCES_DIR);

  std::string urdf_file_path = (res_dir / "baxter.urdf").string();
  
  kinlib::KinematicsSolver kin_solver;
  op_result = kin_solver.loadManipulator(urdf_file_path, "base", "left_gripper_base");
  
  if(!op_result)
  {
    std::cout << "Error loading manipulator URDF";
    return 1;
  }
  
  kinlib::Manipulator manip = kin_solver.getManipulator();
  
  /*
  const int nrolls = 10000;
  const int nintervals = 10;
  
  int p[nintervals] = {};
  
  for(int i = 0; i < nrolls; i++)
  {
    double number = distribution(generator);
    ++p[int(nintervals * number)];
  }
  
  std::cout << std::fixed; std::cout.precision(1);
  
  for(int i = 0; i < nintervals; i++)
  {
    std::cout << float(i)/nintervals << "-" << float(i+1)/nintervals << ": ";
    std::cout << p[i] << std::endl;
  }
  */
  
  std::cout << std::fixed; std::cout.precision(5);
  
  const int ntests = 1000;
  double avg_time = 0;
  
  Eigen::VectorXd rand_joint_vals_i(manip.joint_count_);
  Eigen::VectorXd rand_joint_vals_f(manip.joint_count_);
  Eigen::VectorXd joint_vals_inc(manip.joint_count_);
  
  Eigen::Matrix4d ee_g_i;
  Eigen::Matrix4d ee_g_f;
  
  eigen_ext::DualQuat dq_current;
  eigen_ext::DualQuat dq_next;
  eigen_ext::DualQuat dq_goal;
  
  double sclerp_interpolation_factor = 0.01;
  
  double joint_range[manip.joint_count_] = {};
  
  for(int itr = 0; itr < manip.joint_count_; itr++)
  {
    joint_range[itr] = manip.joint_limits_[itr].upper_limit_ - manip.joint_limits_[itr].lower_limit_;
  }
  
  for(int test_itr = 0; test_itr < ntests; test_itr++)
  {
    std::cout << std::string(100, '=') << std::endl;
    std::cout << "Test " << test_itr+1 << " :\n";
  
    /*
    std::cout << std::string(100, '=') << std::endl;
    std::cout << "Test " << std::noshowpos << test_itr+1 << " :\n";
    std::cout << std::string(100, '-') << std::endl << std::showpos;
    */
  
    double rand_vals[manip.joint_count_] = {};
    
    //std::cout << "Initial Angles :    ";
    for(int jnt_itr = 0; jnt_itr < manip.joint_count_; jnt_itr++)
    {
      rand_vals[jnt_itr] = distribution(generator);
      rand_joint_vals_i(jnt_itr) =
          manip.joint_limits_[jnt_itr].lower_limit_ + (rand_vals[jnt_itr] * joint_range[jnt_itr]);
      //std::cout << rand_joint_vals_i(jnt_itr) << "    ";
    }
    //std::cout << std::endl;
    
    //std::cout << "Final Angles   :    ";
    for(int jnt_itr = 0; jnt_itr < manip.joint_count_; jnt_itr++)
    {
      rand_vals[jnt_itr] = distribution(generator);
      rand_joint_vals_f(jnt_itr) =
          manip.joint_limits_[jnt_itr].lower_limit_ + (rand_vals[jnt_itr] * joint_range[jnt_itr]);
      //std::cout << rand_joint_vals_f(jnt_itr) << "    ";
    }
    //std::cout << std::endl;
    
    kin_solver.getFK(rand_joint_vals_i, ee_g_f);
    dq_goal = eigen_ext::DualQuat::transformationToDualQuat(ee_g_f);
    
    auto begin = std::chrono::high_resolution_clock::now();
    
    // Control Loop Start
    
    kin_solver.getFK(rand_joint_vals_i, ee_g_i);
    dq_current = eigen_ext::DualQuat::transformationToDualQuat(ee_g_i);
    dq_next = eigen_ext::DualQuat::dualQuatInterpolation(dq_current, dq_goal, sclerp_interpolation_factor);
    kin_solver.getResolvedMotionRateControlStep(dq_current, dq_next, rand_joint_vals_i, joint_vals_inc);
    
    // Control Loop End
    
    auto end = std::chrono::high_resolution_clock::now();
    
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    
    std::cout << "  - Time measured : " << elapsed.count() * 1e-6 << std::endl;
    
    avg_time = avg_time + (elapsed.count() * 1e-6);
    
    std::cout << std::string(100, '-') << std::endl;
  }
  
  avg_time = avg_time / ntests;
  
  std::cout << std::string(100, '=') << std::endl;
  std::cout << std::string(100, '-') << std::endl;
  std::cout << "Average time : " << avg_time << std::endl;
  std::cout << std::string(100, '-') << std::endl;
  std::cout << std::string(100, '=') << std::endl;

  return 0;
}
