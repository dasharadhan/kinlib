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
      jnt_ang_file_path_ = (res_dir / "JointAngles.csv").string();
    }

    kinlib::KinematicsSolver kin_solver_;
    std::string urdf_file_path_;
    std::string jnt_ang_file_path_;
};

TEST_F(KinlibTest, KinSolverLoadManipulator)
{
  EXPECT_EQ(
      kin_solver_.loadManipulator(urdf_file_path_, "base", "left_gripper_base"),
      true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
