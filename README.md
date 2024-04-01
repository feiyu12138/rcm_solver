# Remote Center of Motion for UR5-dVrk
## INIT PARAM
```bash
k_task: task gain default = 1.5
k_rcm: rcm gain default = 5
task_null: deprecated
k_p: default = 1
k_i: deprecated
k_d: deprecated
position_only: deprecated
k_pos: gain of position requirement, default = 1.5
k_ori: gain of orientation requirement, default = 1.5
dt: virtual motion time, used for trajectory msg publish and ik solver, default = 0.2 (should align with real communication frequency)
thres: error norm threshold for early stop, default=1e-3(0.1mm)
max_iter: max iteration number for ik solver
ur_prefix: string
tool_prefix string
```
## USAGE
```bash
Eigen::VectorXd q_vec(9);
Eigen::VectorXd q_new(9);
# retrieve q_vec from joint state
robot_description_name = "robot_description";
# retrieve robot_description from node parameters
RCM::RCMSolver rcm_solver(node, robot_description_name);
# retrieve rcm_pose from geometry_msgs
# retrieve target_pose from geometery_msgs
rcm_solver.setRCMPoint(rcm_pose);
# rcm_solver.setDefaultRCM(q_vec)
rcm_solver.setDesiredPose(target_pose);
q_new = rcm_solver.solveIK(q_vec);
# post-process q_new
```
## TODO
```bash
Joint limit detection
Singularity detection
```

