/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Alexander Buchegger
 *                      Institute for Software Technology,
 *                      Graz University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <tedusar_cartesian_arm_teleop/cartesian_arm_teleop_node.h>
#include <stdexcept>

namespace tedusar_cartesian_arm_teleop
{

CartesianArmTeleopNode::CartesianArmTeleopNode()
    : repeat_vel_commands_(false)
{
}

void CartesianArmTeleopNode::init()
{
    loadParameters();
    ROS_INFO_STREAM("Controller: " << parameters_.controller_);

    cmd_vel_.header.frame_id = parameters_.frame_id_;

    cmd_generator_timer_ = nh_.createTimer(ros::Duration(1.0 / parameters_.publication_rate_),
        &CartesianArmTeleopNode::cmdGeneratorTimerCB, this, false);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cartesian_controller/arm_cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
        &CartesianArmTeleopNode::joyCB, this);

    if (parameters_.have_gripper_)
    {
        gripper_command_ac_.reset(new GripperCommandActionClient("/gripper_controller/gripper_cmd", true));
        grasping_done_pub_ = nh_.advertise<std_msgs::Empty>("grasping_done", 1);
    }
}

void CartesianArmTeleopNode::loadParameters()
{
    ros::NodeHandle private_nh("~");
#define GOP(key, default_value) getOptionalParameter(private_nh, #key, parameters_.key##_, default_value)
#define GRP(key) getRequiredParameter(private_nh, #key, parameters_.key##_)

    GOP(publication_rate, 20.0);
    GRP(frame_id);

    GOP(std_translation_vel, 0.01);
    GOP(std_rotation_vel, 0.1);
    GOP(turbo_factor, 2.0);

    GOP(have_gripper, true);
    GOP(gripper_grasp_position, 0.0);
    GOP(gripper_grasp_effort, 10.0);
    GOP(gripper_release_position, 0.1);
    GOP(gripper_release_effort, 0.0);

    GRP(controller);
    GRP(button_translation_deadman_switch);
    GRP(button_rotation_deadman_switch);
    GRP(button_turbo);

    if (parameters_.have_gripper_)
    {
        GRP(button_gripper_grasp);
        GRP(button_gripper_release);
        GRP(button_grasp_done);
    }

    GRP(axis_translation_x);
    GRP(axis_translation_y);
    GRP(axis_translation_z);
    GRP(axis_rotation_x);
    GRP(axis_rotation_y);
    GRP(axis_rotation_z);
#undef GOP
#undef GRP
}

void CartesianArmTeleopNode::cmdGeneratorTimerCB(const ros::TimerEvent & e)
{
    if (repeat_vel_commands_)
        sendCmdVel();
}

void CartesianArmTeleopNode::joyCB(const sensor_msgs::Joy::ConstPtr & joy)
{
    bool have_vel_commands = false;

    double turbo = joy->buttons.at(parameters_.button_turbo_) ? parameters_.turbo_factor_ : 1.0;

    if (joy->buttons.at(parameters_.button_translation_deadman_switch_) && !joy->buttons.at(parameters_.button_rotation_deadman_switch_))
    {
      have_vel_commands = true;
      cmd_vel_.twist.linear.x = parameters_.std_translation_vel_ * turbo * joy->axes.at(parameters_.axis_translation_x_);
      cmd_vel_.twist.linear.y = parameters_.std_translation_vel_ * turbo * joy->axes.at(parameters_.axis_translation_y_);
      cmd_vel_.twist.linear.z = parameters_.std_translation_vel_ * turbo * joy->axes.at(parameters_.axis_translation_z_);
    }
    else
    {
      cmd_vel_.twist.linear.x = 0;
      cmd_vel_.twist.linear.y = 0;
      cmd_vel_.twist.linear.z = 0;
    }

    if (!joy->buttons.at(parameters_.button_translation_deadman_switch_) && joy->buttons.at(parameters_.button_rotation_deadman_switch_))
    {
        have_vel_commands = true;
        cmd_vel_.twist.angular.x = parameters_.std_rotation_vel_ * turbo * joy->axes.at(parameters_.axis_rotation_x_);
        cmd_vel_.twist.angular.y = parameters_.std_rotation_vel_ * turbo * joy->axes.at(parameters_.axis_rotation_y_);
        cmd_vel_.twist.angular.z = parameters_.std_rotation_vel_ * turbo * joy->axes.at(parameters_.axis_rotation_z_);
    }
    else
    {
        cmd_vel_.twist.angular.x = 0;
        cmd_vel_.twist.angular.y = 0;
        cmd_vel_.twist.angular.z = 0;
    }

    if (parameters_.have_gripper_ && joy->buttons.at(parameters_.button_translation_deadman_switch_) &&
            joy->buttons.at(parameters_.button_rotation_deadman_switch_))
    {
        if (joy->buttons.at(parameters_.button_gripper_release_))
            sendGripperCommand(parameters_.gripper_release_position_, parameters_.gripper_release_effort_);
        else if (joy->buttons.at(parameters_.button_gripper_grasp_))
            sendGripperCommand(parameters_.gripper_grasp_position_, parameters_.gripper_grasp_effort_);
        else if (joy->buttons.at(parameters_.button_grasp_done_))
            sendGraspDone();
    }

    if (have_vel_commands)
        cmd_vel_.header.stamp = joy->header.stamp;

    if (have_vel_commands || repeat_vel_commands_)
    {
      sendCmdVel();
      repeat_vel_commands_ = have_vel_commands;
    }
}

void CartesianArmTeleopNode::sendCmdVel()
{
    cmd_vel_pub_.publish(cmd_vel_);
}

void CartesianArmTeleopNode::sendGripperCommand(double position, double max_effort)
{
    if (gripper_command_ac_)
    {
        control_msgs::GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;
        gripper_command_ac_->sendGoal(goal);
    }
}

double CartesianArmTeleopNode::limit(double value, double min, double max)
{
    if (value < min)
      return min;
    if (value > max)
      return max;
    return value;
}

void CartesianArmTeleopNode::sendGraspDone()
{
    std_msgs::Empty msg;
    grasping_done_pub_.publish(msg);
}

template <typename T>
void CartesianArmTeleopNode::getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, T & value)
{
    if (!private_nh.getParam(key, value))
    {
        ROS_FATAL_STREAM("Missing required node parameter " << key);
        throw std::runtime_error("Missing required node parameter " + key);
    }
}

template <typename T>
void CartesianArmTeleopNode::getOptionalParameter(ros::NodeHandle & private_nh, const std::string & key, T & value, T default_value)
{
    if (!private_nh.getParam(key, value))
    {
        value = default_value;
        ROS_WARN_STREAM("Missing optional node parameter " << key << ", using default value " << default_value);
    }
}




}


int main(int argc, char ** argv)
{
    try
    {
        ros::init(argc, argv, "cartesian_arm_teleop");
        tedusar_cartesian_arm_teleop::CartesianArmTeleopNode node;
        node.init();

        ros::spin();
    }
    catch (std::exception & ex)
    {
        ROS_ERROR("Unhandled exception: %s", ex.what());
        return 1;
    }
    catch (...)
    {
        ROS_ERROR("Unhandled exception!");
        return 1;
    }

    return 0;
}
