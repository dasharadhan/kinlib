#include <kinlib/kinlib_kinematics.h>
#include <kinlib_test/config.h>
#include <boost/filesystem/path.hpp>
#include <armadillo>
#include <fstream>
#include <gtest/gtest.h>

class KinlibTest : public testing::Test
{
  protected:
    void SetUp() override
    {
      boost::filesystem::path res_dir(KINLIB_TEST_RESOURCES_DIR);
      
      urdf_file_path_ = (res_dir / "baxter.urdf").string();
      jnt_ang_file_path_ = (res_dir / "joint_angles.csv").string();
      matlab_fk_results_path_ = (res_dir / "matlab_fk_results.csv").string();
      screw_param_check_g_i_path_ = (res_dir / "screw_param_check_g_i.csv").string();
      screw_param_check_g_f_path_ = (res_dir / "screw_param_check_g_f.csv").string();
      screw_param_results_path_ = (res_dir / "screw_params.csv").string();


      joint_names_.push_back("left_s0");
      joint_names_.push_back("left_s1");
      joint_names_.push_back("left_e0");
      joint_names_.push_back("left_e1");
      joint_names_.push_back("left_w0");
      joint_names_.push_back("left_w1");
      joint_names_.push_back("left_w2");
    }

    kinlib::KinematicsSolver kin_solver_;
    kinlib::Manipulator manip_;
    std::string urdf_file_path_;
    std::string jnt_ang_file_path_;
    std::string matlab_fk_results_path_;
    std::string screw_param_check_g_i_path_;
    std::string screw_param_check_g_f_path_;
    std::string screw_param_results_path_;
    std::vector<std::string> joint_names_;
};

TEST_F(KinlibTest, KinSolverLoadManipulator)
{
  // Check if manipulator loads
  EXPECT_EQ(
      kin_solver_.loadManipulator(urdf_file_path_, "base", "left_gripper_base"),
      true);

  // Check if manipulator is loaded properly
  manip_ = kin_solver_.getManipulator();
  EXPECT_EQ(manip_.joint_count_, 7);
  for(int i = 0; i < 7; i++)
  {
    EXPECT_EQ(manip_.joint_types_[i], kinlib::JointType::Revolute);
    EXPECT_EQ(manip_.joint_names_[i], joint_names_[i]);
  }
}

TEST_F(KinlibTest, KinSolverFK)
{
  // Load Manipulator Properties
  EXPECT_EQ(
      kin_solver_.loadManipulator(urdf_file_path_, "base", "left_gripper_base"),
      true);

  // Forward Kinematics check with MATLAB results
  arma::Mat<double> jnt_angles;
  arma::Mat<double> matlab_fk_results;
  jnt_angles.load(jnt_ang_file_path_);
  matlab_fk_results.load(matlab_fk_results_path_);

  Eigen::VectorXd jnt_val(7);
  Eigen::Matrix4d ee_g;
  for(unsigned int i = 0; i < jnt_angles.n_rows; i++)
  {
    for(int j = 0; j < 7; j++)
    {
      jnt_val(j) = jnt_angles(i,j);
    }

    ASSERT_EQ(kin_solver_.getFK(jnt_val, ee_g),
              kinlib::ErrorCodes::OPERATION_SUCCESS);

    bool g_check = true;

    for(int j = 0; j < 4; j++)
    {
      for(int k = 0; k < 4; k++)
      {
        if((matlab_fk_results((i*4)+j,k) - ee_g(j,k)) > 1.0e-5)
        {
          g_check = false;
          break;
        }
      }

      if(!g_check)
      {
        break;
      }
    }

    ASSERT_EQ(g_check, true);
  }
}

TEST_F(KinlibTest, GetScrewParamCheck)
{
  // Load rigid body poses and screw parameters
  arma::Mat<double> rigid_body_poses_i;
  arma::Mat<double> rigid_body_poses_f;
  arma::Mat<double> matlab_screw_params;
  rigid_body_poses_i.load(screw_param_check_g_i_path_);
  rigid_body_poses_f.load(screw_param_check_g_f_path_);
  matlab_screw_params.load(screw_param_results_path_);

  // Check screw params
  // First 25 poses comprise of pure translation motion
  // Next 25 poses comprise of pure rotational motion
  // Next 5 poses comprise of same initial and final poses (No motion)
  // Remaining poses comprise of general screw motion

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
