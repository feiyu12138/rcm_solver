// Luoxin 2024-03-20
// RCMSolver.hpp
#ifndef RCM_SOLVER_HPP
#define RCM_SOLVER_HPP
#include <rclcpp/rclcpp.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>

namespace RCM{
class RCMSolver
{
public:
    RCMSolver(rclcpp::Node::SharedPtr node, const std::string& robot_description_name);

    void setRCMPoint(const geometry_msgs::msg::Pose& rcm_pose);
    void setDesiredPose(const geometry_msgs::msg::Pose& target_pose);
    void setDefaultRCM(Eigen::VectorXd q_vec);
    Eigen::VectorXd solveIK(Eigen::VectorXd q_vec);

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<std::string> joint_names_;
    geometry_msgs::msg::Pose rcm_pose_;
    geometry_msgs::msg::Pose target_pose_;
    std::string robot_description_;
    
    KDL::Tree tree_;
    // KDL Chains
    KDL::Chain chain_;
    KDL::Chain shaft_chain_;
    KDL::Chain rotation_chain_;


    // KDL Solvers
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_shaft_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_rotation_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_shaft_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_rotation_solver_;

    double k_task_=1.5;
    double k_rcm_=5.0;
    bool task_null_=false;
    double k_p_=1.0;
    double k_i_=0.0;
    double k_d_=0.0;
    bool position_only_;
    double k_pos_=1.5;
    double k_ori_=1.5;
    double dt_=0.2;
    double thres_=1e-3;
    int max_iter_=10;
    float offset_=0.0;
    std::string ur_prefix_;
    std::string tool_prefix_;

    // Gain Matrix
    Eigen::MatrixXd K_;

    void initializeKDLChain();

    std::pair<Eigen::VectorXd, Eigen::VectorXd> calcAndUpdate(Eigen::VectorXd q_vec);
    
    Eigen::Matrix3d skew(Eigen::Vector3d v);
    
    KDL::JntArray VectorToJnt(const Eigen::VectorXd& eigen_vector,int size=9);
    
    KDL::Frame MatrixToFrame(const Eigen::Matrix4d& eigen_matrix);
    
    Eigen::Vector3d FrameToVec3d(const KDL::Frame& frame);
    
    Eigen::Matrix4d FrameToMatrix(const KDL::Frame& frame);

    geometry_msgs::msg::Pose FrameToPoseMsg(const KDL::Frame& frame);
    
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix, double tolerance = 1e-4);
    
    Eigen::VectorXd get_task_error(Eigen::Matrix4d T_desired, Eigen::Matrix4d T_current);

    Eigen::Matrix4d poseToTransform(geometry_msgs::msg::Pose pose);
    
    // geometry_msgs::msg::Pose matrixToPoseMsg(const Eigen::Matrix4d& transformationMatrix)
    
    std::pair<Eigen::Vector3d, double> findClosestPointOnSegment(Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b);
    
    Eigen::MatrixXd calc_rcm_jacobian_kdl(
        KDL::Jacobian J_shaft,
        KDL::Jacobian J_rotation,
        Eigen::Vector3d shaft_point,
        Eigen::Vector3d rotation_point,
        double lambda);
};
} // namespace RCM

#endif // RCM_SOLVER_HPP