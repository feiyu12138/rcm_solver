// Luoxin 2024-03-20
#include <rcm/RCMSolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace RCM{

KDL::JntArray 
RCMSolver::VectorToJnt(const Eigen::VectorXd& eigen_vector,int size) {
    KDL::JntArray jnt_array(size);

    for (int i = 0; i < size; ++i) {
        jnt_array(i) = eigen_vector(i);
    }

    return jnt_array;
}
Eigen::Matrix3d 
RCMSolver::skew(Eigen::Vector3d v){
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  return m;
}
KDL::Frame 
RCMSolver::MatrixToFrame(const Eigen::Matrix4d& eigen_matrix) {
    // Extract rotation matrix
    Eigen::Matrix3d rotation_matrix = eigen_matrix.block<3, 3>(0, 0);
    KDL::Rotation kdl_rotation(
        rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
        rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
        rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

    // Extract translation vector
    KDL::Vector kdl_translation(eigen_matrix(0, 3), eigen_matrix(1, 3), eigen_matrix(2, 3));

    // Construct and return the KDL::Frame
    return KDL::Frame(kdl_rotation, kdl_translation);
}
Eigen::Vector3d 
RCMSolver::FrameToVec3d(const KDL::Frame& frame) {
    // Extract the position part of the KDL::Frame
    double x = frame.p.x();
    double y = frame.p.y();
    double z = frame.p.z();

    // Create an Eigen::Vector3d with the extracted position
    Eigen::Vector3d eigenVector(x, y, z);

    return eigenVector;
}
Eigen::Matrix4d 
RCMSolver::poseToTransform(geometry_msgs::msg::Pose pose){
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Isometry3d T;
  T.setIdentity();
  T.rotate(q);
  T.pretranslate(p);
  return T.matrix();
}
Eigen::Matrix4d 
RCMSolver::FrameToMatrix(const KDL::Frame& frame) {
    Eigen::Matrix4d eigen_matrix;

    // Convert the rotation part
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigen_matrix(i, j) = frame.M(i, j);
        }
    }

    // Convert the translation part
    eigen_matrix(0, 3) = frame.p.x();
    eigen_matrix(1, 3) = frame.p.y();
    eigen_matrix(2, 3) = frame.p.z();

    // Fill the bottom row of the homogeneous transformation matrix
    eigen_matrix(3, 0) = 0.0;
    eigen_matrix(3, 1) = 0.0;
    eigen_matrix(3, 2) = 0.0;
    eigen_matrix(3, 3) = 1.0;

    return eigen_matrix;
}
geometry_msgs::msg::Pose 
RCMSolver::FrameToPoseMsg(const KDL::Frame& frame){
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  pose.orientation.x = x;
  pose.orientation.y = y;
  pose.orientation.z = z;
  pose.orientation.w = w;
  return pose;
};
std::pair<Eigen::Vector3d, double> 
RCMSolver::findClosestPointOnSegment(Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b){
  // find the closest point on a line segment
  Eigen::Vector3d ab = b - a;
  double t = (p - a).dot(ab) / ab.dot(ab);
  t = std::max(-1.0, std::min(1.0, t));
  auto closest_point = a + t * ab;
  return std::make_pair(closest_point, t);
}
Eigen::VectorXd 
RCMSolver::get_task_error(Eigen::Matrix4d T_desired, Eigen::Matrix4d T_current){
  Eigen::Matrix3d R_des = T_desired.block<3,3>(0,0);
  Eigen::Matrix3d R_cur = T_current.block<3,3>(0,0);
  Eigen::Vector3d p_des = T_desired.block<3,1>(0,3);
  Eigen::Vector3d p_cur = T_current.block<3,1>(0,3);

  Eigen::Matrix3d R_err = R_des * R_cur.transpose();
  Eigen::Vector3d p_err = p_des - p_cur;

  // Clamp the argument of acos to the valid range [-1, 1] to avoid NaN
  double traceValue = (R_err.trace() - 1) / 2;
  traceValue = std::max(-1.0, std::min(1.0, traceValue)); // Clamping
  double theta = acos(traceValue);

  Eigen::VectorXd e(6);
  e.head(3) = p_err;

  if (std::abs(theta) > 1e-6) { // Check if theta is not too small
    Eigen::Matrix3d W = (R_err - R_err.transpose()) / (2 * sin(theta));
    Eigen::Vector3d w;
    w << W(2,1), W(0,2), W(1,0);
    w = theta * w / (w.norm() + 1e-6); // Ensure not dividing by a very small number
    e.tail(3) = w;
  } else {
    // For very small theta, set rotational error to zero or use an alternative approximation
    e.tail(3).setZero();
  }

  return e;
}
Eigen::MatrixXd 
RCMSolver::pseudoInverse(const Eigen::MatrixXd &matrix, double tolerance){
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::MatrixXd singularValuesInverse(matrix.cols(), matrix.rows());
  singularValuesInverse.setZero();
  for (std::ptrdiff_t i = 0; i < singularValues.size(); i++) {
    if (singularValues(i) > tolerance || singularValues(i) < -tolerance){
      singularValuesInverse(i,i) = 1.0 / singularValues(i);
    }
    else{
      std::cout << "Singularity" << std::endl;
      singularValuesInverse(i,i) = 0.0;
    }
  }
  return svd.matrixV() * singularValuesInverse * svd.matrixU().adjoint();
}
RCMSolver::RCMSolver(rclcpp::Node::SharedPtr node, const std::string& robot_description_name)
    : node_(node)
{
    node_->get_parameter("robot_description", robot_description_);
    node_->get_parameter("ur_prefix",ur_prefix_);
    node_->get_parameter("tool_prefix",tool_prefix_);
    initializeKDLChain();
    node_->get_parameter("k_task", k_task_);
    node_->get_parameter("k_rcm", k_rcm_);
    node_->get_parameter("task_null", task_null_);
    node_->get_parameter("k_p",k_p_);
    node_->get_parameter("k_i",k_i_);
    node_->get_parameter("k_d",k_d_);
    node_->get_parameter("position_only",position_only_);
    node_->get_parameter("k_pos",k_pos_);
    node_->get_parameter("k_ori",k_ori_);
    node_->get_parameter("dt",dt_);
    node_->get_parameter("thres",thres_);
    node_->get_parameter("max_iter",max_iter_);
    node_->get_parameter("offset",offset_);
    
    Eigen::MatrixXd K(9,9);
    K << k_task_ * k_pos_ * Eigen::MatrixXd::Identity(3,3),  
        Eigen::MatrixXd::Zero(3,6),
        Eigen::MatrixXd::Zero(3,3),
        k_task_ * k_ori_ * Eigen::MatrixXd::Identity(3,3),
        Eigen::MatrixXd::Zero(3,3),
        Eigen::MatrixXd::Zero(3,6), 
        k_rcm_ * Eigen::MatrixXd::Identity(3,3);
    K_ = K;
}
void RCMSolver::setRCMPoint(const geometry_msgs::msg::Pose& rcm_pose)
{
    rcm_pose_ = rcm_pose;
}
void RCMSolver::setDefaultRCM(Eigen::VectorXd q_vec){
    KDL::JntArray q_cur_shaft(7);
    q_cur_shaft = VectorToJnt(q_vec,7);
    KDL::Frame F_cur_shaft;
    fk_shaft_solver_->JntToCart(q_cur_shaft, F_cur_shaft);
    KDL::Vector offset_vec(0,0,offset_);
    Eigen::MatrixXd tempt_matrix = FrameToMatrix(F_cur_shaft);
    F_cur_shaft = F_cur_shaft * KDL::Frame(KDL::Rotation::Identity(), offset_vec);
    rcm_pose_ = FrameToPoseMsg(F_cur_shaft);
}
void RCMSolver::setDesiredPose(const geometry_msgs::msg::Pose& target_pose)
{
    target_pose_ = target_pose;
}
Eigen::VectorXd RCMSolver::solveIK(Eigen::VectorXd q_vec)
{
    auto result = calcAndUpdate(q_vec);
    return result.first; // Return only the joint positions
}
Eigen::MatrixXd 
RCMSolver::calc_rcm_jacobian_kdl(
  KDL::Jacobian J_shaft,
  KDL::Jacobian J_rotation,
  Eigen::Vector3d shaft_point,
  Eigen::Vector3d rotation_point,
  double lambda)
{ 
  Eigen::MatrixXd J_shaft_matrix(6,9);
  J_shaft_matrix.setZero();
  Eigen::MatrixXd J_rotation_matrix(6,9);
  J_rotation_matrix.setZero();
  J_shaft_matrix.block<6, 7>(0, 0) = J_shaft.data;
  J_rotation_matrix.block<6, 7>(0, 0) = J_rotation.data;


  Eigen::MatrixXd J_rcm_left(6,9);
  Eigen::MatrixXd J_rcm_left_pos(3,9);
  Eigen::MatrixXd J_rcm_right(3,1);
  J_rcm_right = rotation_point - shaft_point;
  J_rcm_left = J_shaft_matrix + lambda * (J_rotation_matrix - J_shaft_matrix);
  // get top 3 rows of J_rcm_left
  J_rcm_left_pos = J_rcm_left.topRows(3);
  Eigen::MatrixXd J_rcm(J_rcm_left_pos.rows(), J_rcm_left_pos.cols() + J_rcm_right.cols());
  J_rcm << J_rcm_left_pos, J_rcm_right;
  return J_rcm;
}
void RCMSolver::initializeKDLChain()
{
    if (!kdl_parser::treeFromString(robot_description_, tree_)){
    RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree");
    return;
    }
    tree_.getChain(ur_prefix_ + "base_link",tool_prefix_ + "tcp0",chain_);
    tree_.getChain(ur_prefix_ + "base_link",tool_prefix_ + "shaft",shaft_chain_);
    tree_.getChain(ur_prefix_ + "base_link",tool_prefix_ + "rotation",rotation_chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    fk_shaft_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(shaft_chain_);
    fk_rotation_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(rotation_chain_);
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    jac_shaft_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(shaft_chain_);
    jac_rotation_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(rotation_chain_);

}
std::pair<Eigen::VectorXd, Eigen::VectorXd> RCMSolver::calcAndUpdate(Eigen::VectorXd q_vec)
{
    Eigen::MatrixXd J_rcm_Matrix;
    Eigen::MatrixXd J_task_Matrix;
    Eigen::VectorXd dq_rcm(10);
    Eigen::VectorXd dq_task(10);
    Eigen::VectorXd dq(10);
    Eigen::VectorXd e(9);
    Eigen::VectorXd e_task(6);
    Eigen::VectorXd e_rcm(3);
    Eigen::MatrixXd J_pinv;
    Eigen::VectorXd q_new(9);
    Eigen::VectorXd error(6);
    error.setZero();
    KDL::Jacobian J_task(9);
    KDL::Jacobian J_rotation(7);
    KDL::Jacobian J_shaft(7);
    KDL::Frame F_cur_frame;
    KDL::Frame F_cur_shaft;
    KDL::Frame F_cur_rotation;

    Eigen::Vector3d rotation_point;
    Eigen::Vector3d rcm_point(
      rcm_pose_.position.x, 
      rcm_pose_.position.y, 
      rcm_pose_.position.z);;
    Eigen::Vector3d shaft_point;

    KDL::JntArray q_cur(9);
    KDL::JntArray q_shaft(7);
    KDL::JntArray q_rotation(7);

    Eigen::Matrix4d F_target = poseToTransform(target_pose_);
    Eigen::Matrix4d F_cur;

    double e_task_norm;

    std::pair<Eigen::Vector3d, double> trocar;
    Eigen::Vector3d trocar_point;
    double lambda;
    std::pair<Eigen::VectorXd, Eigen::VectorXd> result;
    q_new.setZero();
    for (int i = 0; i < max_iter_; i++)
    {
        q_cur = VectorToJnt(q_vec);
        q_shaft = VectorToJnt(q_vec,7);
        q_rotation = VectorToJnt(q_vec,7);
        fk_solver_->JntToCart(q_cur, F_cur_frame);
        fk_shaft_solver_->JntToCart(q_shaft, F_cur_shaft);
        fk_rotation_solver_->JntToCart(q_rotation, F_cur_rotation);
        F_cur = FrameToMatrix(F_cur_frame);
        e_task = get_task_error(F_target, F_cur);
        if (e_task_norm = e_task.norm(); e_task_norm > 0.1){
            e_task = e_task / (1e-7 + e_task_norm) * 0.1;
        }
        rotation_point = FrameToVec3d(F_cur_rotation);
        shaft_point = FrameToVec3d(F_cur_shaft);
        trocar = findClosestPointOnSegment(rcm_point, shaft_point, rotation_point);
        trocar_point = trocar.first;
        lambda = trocar.second;
        jac_solver_->JntToJac(q_cur,J_task);
        jac_shaft_solver_->JntToJac(q_shaft,J_shaft);
        jac_rotation_solver_->JntToJac(q_rotation,J_rotation);
        J_rcm_Matrix = calc_rcm_jacobian_kdl(J_shaft,J_rotation, shaft_point,rotation_point, lambda);
        Eigen::MatrixXd J(J_rcm_Matrix.rows() + J_task.data.rows(), J_rcm_Matrix.cols());
        J << J_task.data, Eigen::VectorXd::Zero(J_task.data.rows()), J_rcm_Matrix;
        e_rcm = rcm_point - trocar_point;
        if (auto e_rcm_norm = e_rcm.norm(); e_rcm_norm > 0.5){
            e_rcm = e_rcm / e_rcm_norm * 0.5;
        }
        e << e_task, e_rcm;
        J_pinv = pseudoInverse(J);
        dq = J_pinv * K_ * e;

        for (int i =0; i < 9; i++){
            q_new(i) = q_vec(i) + dq(i) * dt_;
        }
        if (e.norm() < thres_){
            break;
        }
        q_vec = q_new;
    }
    result = std::make_pair(q_new, e);

    return result;
}
} // namespace RCM