#include <iostream>
#include <fstream>
#include <array>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>
// void saveMotionPlanToCSV(const std::string& filename, const std::vector<Eigen::VectorXd>& motion_plan_result) {
//     std::ofstream file(filename);

//     if (!file.is_open()) {
//         throw std::runtime_error("Unable to open file for writing: " + filename);
//     }

//     // Iterate through the motion plan result
//     for (const auto& step : motion_plan_result) {
//         for (int i = 0; i < step.size(); ++i) {
//             file << step[i];  // Write each element
//             if (i < step.size() - 1) {
//                 file << ",";  // Add a comma between elements
//             }
//         }
//         file << "\n";  // Newline at the end of each row
//     }

//     file.close();
//     std::cout << "[SUCCESS] Motion plan saved to: " << filename << std::endl;
// }

void saveMotionPlanToCSV(const std::vector<Eigen::Matrix4d>& motion_plan, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < motion_plan.size(); ++i) {
        const Eigen::Matrix4d& matrix = motion_plan[i];
        
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                file << matrix(row, col);
                if (col < 3) file << ",";  // Add commas between columns
            }
            file << std::endl;
        }
       
    }

    file.close();
    std::cout << "Motion plan saved to " << filename << std::endl;
}
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


int main()
{
    // User Guided Motion Planner
  Eigen::MatrixXd recorded_demo = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "Demonstrations/panda_demo/real_Panda_ee_traj.csv");
  Eigen::MatrixXd object_poses = loadCSV<Eigen::MatrixXd>(std::string(KINLIB_RESOURCES_DIR) + "Demonstrations/panda_demo/Real_panda_obj_pos.csv");

  std::vector<Eigen::Matrix4d> recorded_ee_traj;
  std::vector<Eigen::Matrix4d> obj_poses;

  for(int i = 0; i < recorded_demo.rows()/4; i++)
  {
    Eigen::Matrix4d g = recorded_demo.block<4,4>((i*4),0);
    recorded_ee_traj.push_back(g);
  }

  for(int i = 0; i < object_poses.rows()/4; i++)
  {
    Eigen::Matrix4d g = object_poses.block<4,4>((i*4),0);
    obj_poses.push_back(g);
  }

  std::vector<Eigen::Matrix4d> filtered_ee_traj;
  kinlib::filterSE3Sequence(recorded_ee_traj,filtered_ee_traj);


  kinlib::Demonstration demo = kinlib::saveDemonstration(filtered_ee_traj,obj_poses,0.2);
  
  // Guiding 
  
  for(int i = 0; i < demo.guiding_poses.size(); i++)
  {
    std::cout << "\nGuiding poses associated with object " << i+1 << '\n';
    for(int j = 0; j < demo.guiding_poses[i].size(); j++)
    {
      std::cout << demo.guiding_poses[i][j] << '\n';
    }
  }


  std::cout << "demo TASK object" << '\n';
  std::cout << demo.task_instance.object_poses[0] << '\n';
  std::cout << demo.task_instance.object_poses[1] << '\n';
  std::cout << "demo TASK object finished" << '\n';
  kinlib::TaskInstance new_task_instance;


  new_task_instance.object_poses = demo.task_instance.object_poses;

  new_task_instance.object_poses[0](0,3) = new_task_instance.object_poses[0](0,3) -0.1;
  new_task_instance.object_poses[0](1,3) = new_task_instance.object_poses[0](1,3) ;
  new_task_instance.object_poses[0](2,3) = new_task_instance.object_poses[0](2,3); 

  new_task_instance.object_poses[1](0,3) = new_task_instance.object_poses[1](0,3) - 0.1; 
  new_task_instance.object_poses[1](1,3) = new_task_instance.object_poses[1](1,3) ;
  new_task_instance.object_poses[1](2,3) = new_task_instance.object_poses[1](2,3);
  std::cout << "NEW TASK INSTANCE" << '\n';
  std::cout << new_task_instance.object_poses[0] << '\n';
  std::cout << new_task_instance.object_poses[1] << '\n';
  std::cout << "NEW TASK INSTANCE FINISHED" << '\n';


  // kinlib::TaskInstance new_task_instance;

  // new_task_instance.object_poses = demo.task_instance.object_poses;

  // new_task_instance.object_poses[0](0,3) = new_task_instance.object_poses[0](0,3) + 0.1;
  // new_task_instance.object_poses[0](1,3) = new_task_instance.object_poses[0](1,3) - 0.05;
  // new_task_instance.object_poses[0](2,3) = new_task_instance.object_poses[0](2,3); 

  // new_task_instance.object_poses[1](0,3) = new_task_instance.object_poses[1](0,3) - 0.1; 
  // new_task_instance.object_poses[1](1,3) = new_task_instance.object_poses[1](1,3) + 0.05;
  // new_task_instance.object_poses[1](2,3) = new_task_instance.object_poses[1](2,3);
  // std::cout << new_task_instance.object_poses[1] << '\n';

  std::vector<Eigen::Matrix4d> new_motion_plan;
  kinlib::ErrorCodes res = kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demo,new_task_instance,new_motion_plan);
  // saveMotionPlanToCSV(new_motion_plan, "real_panda_demo_guiding_pose.csv");
  // for(auto g : new_motion_plan)
  // {
    
  //   std::cout << g << '\n';
    
  // }
  
}
