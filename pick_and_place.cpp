#include <exercise3/functions.h>
#include <std_srvs/Empty.h>

// tf2_ros::TransformListener
#include <tf2_ros/transform_listener.h>

// exit, EXIT_FAILURE
#include <stdlib.h>

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
void rm_SetJointTrajToState(robot_state::RobotState &m_state, std::vector<moveit_msgs::RobotTrajectory> &m_trajectories);
void rm_AddIKPathToTraj(robot_state::RobotState &m_state, const moveit::core::JointModelGroup *m_jmg, Eigen::Isometry3d eigen_tf, const std::string ee_link);
void rm_AddCartesianPathToTraj(moveit::planning_interface::MoveGroupInterface &m_arm, geometry_msgs::Pose m_pose, std::vector<moveit_msgs::RobotTrajectory> &m_trajectories);

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Wait for the simulator to come online and then we reset it
    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset");
    ros::service::call("/lumi_mujoco/reset", srv_reset);

    // Load MoveGroup interface and moveit visual tools
    moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
    moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
    moveit_visual_tools::MoveItVisualTools vis("base_link");
    vis.loadMarkerPub(true);
    vis.deleteAllMarkers();
    vis.trigger();

    // Get Start state
    const robot_state::RobotStatePtr state_ptr = g_arm.getCurrentState(10.0);
    if (!state_ptr)
    {
        ROS_ERROR_STREAM("Cannot get current state");
        return -1;
    }
    robot_state::RobotState state = *state_ptr;

    const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(
        "lumi_arm"); // joint model group used for IK computation

    //-------------------------- --------------------------
    // Read the transformation of the frames named pick and place
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped ee_to_base_pick, ee_to_base_place;
    ee_to_base_pick = tfBuffer.lookupTransform("base_link", "pick", ros::Time(0), ros::Duration(1.0));
    ee_to_base_place = tfBuffer.lookupTransform("base_link", "place", ros::Time(0), ros::Duration(1.0));

    // Check base_link to pick information
    // std::cout << ee_to_base_pick << std::endl;

    //-------------------------- --------------------------
    // Converts a Transform message into an Eigen Transform.
    Eigen::Isometry3d base_pick_tf, base_place_tf;
    // geometry_msgs/TransformStamped -> geometry_msgs/Transform transform
    tf::transformMsgToEigen(ee_to_base_pick.transform, base_pick_tf);
    tf::transformMsgToEigen(ee_to_base_place.transform, base_place_tf);

    // ROS_INFO("EndEffector Link: %s", g_arm.getEndEffectorLink().c_str());


    //-------------------------- --------------------------
    // Config Robot Transform State
    const std::string ee_link = "lumi_ee"; // Name of the end effector link
    const Eigen::Isometry3d arm_to_ee =
        state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
        state.getGlobalLinkTransform(
            ee_link); // Transformation from base link to end effector link

    Eigen::Isometry3d pre_grasp_tf = state.getGlobalLinkTransform("base_link") * base_pick_tf * 
                           Eigen::Translation3d(0.0, 0.0, 0.1) * 
                           Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    
    Eigen::Isometry3d grasp_tf = pre_grasp_tf * Eigen::Translation3d(0.0, 0.0, 0.1);

    Eigen::Isometry3d place_tf = state.getGlobalLinkTransform("base_link") * base_place_tf * 
                           Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    vis.publishAxis(pre_grasp_tf);
    vis.publishAxis(grasp_tf);
    vis.publishAxis(place_tf);
    vis.trigger();

    //-------------------------- --------------------------
    // Robot Trajectory
    std::vector<moveit_msgs::RobotTrajectory> trajectories;

    //-------------------------- --------------------------
    // Solve Inverse Kinematics (IK)
    g_arm.setStartState(state);

    // SET base_link to pre_grasp
    rm_AddIKPathToTraj(state, jmg, pre_grasp_tf, ee_link);
    trajectories.push_back(planToState(g_arm, state));
    // Set RobotTrajectory value (e.g. JointTrajectory) to state
    rm_SetJointTrajToState(state, trajectories);

    //-------------------------- --------------------------
    // Solve Cartesian Path
    g_arm.setStartState(state);
    
    geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(grasp_tf * arm_to_ee.inverse(), grasp_pose);
    // Add Cartesian Path
    rm_AddCartesianPathToTraj(g_arm, grasp_pose, trajectories);
    rm_SetJointTrajToState(state, trajectories);

    //-------------------------- --------------------------
    // Solve Cartesian Path
    g_arm.setStartState(state);
    
    geometry_msgs::Pose pre_grasp_pose;
    tf::poseEigenToMsg(grasp_tf * Eigen::Translation3d(0.0, 0.0, -0.1) * arm_to_ee.inverse(), pre_grasp_pose);
    // Add Cartesian Path
    rm_AddCartesianPathToTraj(g_arm, pre_grasp_pose, trajectories);
    rm_SetJointTrajToState(state, trajectories);

    //-------------------------- --------------------------
    // Solve Inverse Kinematics (IK)
    g_arm.setStartState(state);

    // SET base_link to pre_grasp
    rm_AddIKPathToTraj(state, jmg, place_tf, ee_link);
    trajectories.push_back(planToState(g_arm, state));
    // Set RobotTrajectory value (e.g. JointTrajectory) to state
    rm_SetJointTrajToState(state, trajectories);

    // Visualise all trajectories
    for (const moveit_msgs::RobotTrajectory &t : trajectories)
    {
        vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
    }
    vis.trigger();

    if (askContinue())
    {
        int idx = 0;
        g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, true)));
        // Execute all trajectories
        for (const moveit_msgs::RobotTrajectory &t : trajectories)
        {
            if(idx == 2)
            {
                g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, false)));
            }
            g_arm.execute(trajectoryToPlan(t));
            idx++;
        }
    }
    else
    {
        exit(EXIT_FAILURE);
    }

    return 0;
}

//------------------------------------------------------------------------------
void rm_SetJointTrajToState(robot_state::RobotState &m_state, std::vector<moveit_msgs::RobotTrajectory> &m_trajectories)
{
    // To avoid undefined behavior due to an empty vector
    if (m_trajectories.back().joint_trajectory.points.empty())
    {
        exit(EXIT_FAILURE);
    }
    // Copy those new positions value (JointTrajectory) into state
    m_state.setVariablePositions(
        m_trajectories.back().joint_trajectory.joint_names,
        m_trajectories.back().joint_trajectory.points.back().positions);
}

//------------------------------------------------------------------------------
void rm_AddIKPathToTraj(robot_state::RobotState &m_state, const moveit::core::JointModelGroup *m_jmg, Eigen::Isometry3d eigen_tf, const std::string ee_link)
{
    // bool found_ik = kinematic_state->setFromIK(joint_model_group, eigen_transform, ...);
    if (!m_state.setFromIK(m_jmg, eigen_tf, ee_link))
    {
        ROS_ERROR_STREAM("Cannot set arm position with IK");
        exit(EXIT_FAILURE);
    }
    else
    {
        std::vector<double> joint_values;
        const std::vector<std::string> &joint_names = m_jmg->getVariableNames();
        m_state.copyJointGroupPositions(m_jmg, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
}

//------------------------------------------------------------------------------
void rm_AddCartesianPathToTraj(moveit::planning_interface::MoveGroupInterface &m_arm, geometry_msgs::Pose m_pose, std::vector<moveit_msgs::RobotTrajectory> &m_trajectories)
{
    moveit_msgs::RobotTrajectory rtraj;
    const double d = m_arm.computeCartesianPath({m_pose}, 0.01, 1.4, rtraj);
    if (d < 0.99)
    {
        ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
        exit(EXIT_FAILURE);
    }
    m_trajectories.push_back(rtraj);
}
