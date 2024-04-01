# Remote Center of Motion for UR5-dVrk
## USAGE
```bash
Eigen::VectorXd q_vec(9);
Eigen::VectorXd q_new(9);
# retrieve q_vec from joint state
robot_description_name = "robot_description";
# retrieve robot_description from node parameters
RCM::RCMSolver rcm_solver(node, robot_description_name, joint_names);
# retrieve rcm_pose from geometry_msgs
# retrieve target_pose from geometery_msgs
rcm_solver.setRCMPoint(rcm_pose);
rcm_solver.setDesiredPose(target_pose);
q_new = rcm_solver.solveIK(q_vec);
# post-process q_new
```
