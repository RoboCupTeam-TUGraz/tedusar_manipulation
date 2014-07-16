/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Johannes Maurer
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

#include <tedusar_cartesian_controller/cartesian_controller.hpp>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>

PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianController, controller_interface::ControllerBase)

namespace velocity_controllers
{

    bool CartesianController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &nh)
    {
        std::string robot_description;

        if(!nh.searchParam("robot_description", robot_description))
        {
            ROS_ERROR("Failed to get robot_description parameter");
            return false;
        }

        if (!nh.getParam("root_name", root_name_))
        {
            ROS_ERROR("Failed to get root_name parameter");
            return false;
        }

        if (!nh.getParam("tip_name", tip_name_))
        {
            ROS_ERROR("Failed to get tip_name parameter");
            return false;
        }


        if (!kdl_parser::treeFromParam(robot_description, kdl_tree_)){
            ROS_ERROR("Failed to construct kdl tree from robot description parameter");
            return false;
        }

        // Populate the KDL chain
        if(!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree.");
            return false;
        }

        // Get joint handles for all of the joints in the chain
        for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        {
            joint_handles_.push_back(hw->getHandle(it->getJoint().getName()));
        }

        chain_ik_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        chain_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        vel_cmd_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("arm_cmd_vel", 1, &CartesianController::velCmdCB, this);

        double dead_man_timeout;
        nh.param<double>("dead_man_timeout", dead_man_timeout, 0.2);
        dead_man_timeout_ = ros::Duration(dead_man_timeout);

        nh.param<double>("joint_vel_limit", joint_vel_limit_, 0.3);

        return true;
    }

    void CartesianController::update(const ros::Time& time, const ros::Duration& period)
    {
        if(got_msg_)
        {

            if((time - last_msg_) >= dead_man_timeout_)
            {
                cmd_linear_twist_[0] = 0.0;
                cmd_linear_twist_[1] = 0.0;
                cmd_linear_twist_[2] = 0.0;
                cmd_linear_twist_[3] = 0.0;
                cmd_linear_twist_[4] = 0.0;
                cmd_linear_twist_[5] = 0.0;

                cmd_angular_twist_[0] = 0.0;
                cmd_angular_twist_[1] = 0.0;
                cmd_angular_twist_[2] = 0.0;
                cmd_angular_twist_[3] = 0.0;
                cmd_angular_twist_[4] = 0.0;
                cmd_angular_twist_[5] = 0.0;

                got_msg_ = false;

                for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
                {
                    joint_handles_[i].setCommand(0.0);
                }

                return;
            }

            // get joint positions
            KDL::JntArray  q(joint_handles_.size());
            for(size_t i=0; i<joint_handles_.size(); i++)
            {
                q(i) = joint_handles_[i].getPosition();
            }

            KDL::Frame frame_tip_pose;

            if(chain_fk_solver_->JntToCart(q, frame_tip_pose) < 0)
            {
                ROS_ERROR("Unable to compute forward kinematic.");

                for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
                {
                    joint_handles_[i].setCommand(0.0);
                }

                return;
            }

            KDL::Frame frame_tip_pose_inv = frame_tip_pose.Inverse();

            KDL::Twist linear_twist = frame_tip_pose_inv * cmd_linear_twist_;
            KDL::Twist angular_twist = frame_tip_pose_inv.M * cmd_angular_twist_;

            KDL::Twist twist(linear_twist.vel, angular_twist.rot);

            KDL::JntArray joint_vel(joint_handles_.size());
            if(chain_ik_solver_vel_->CartToJnt(q, twist, joint_vel) < 0)
            {
                ROS_ERROR("Unable to compute cartesian to joint velocity.");

                for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
                {
                    joint_handles_[i].setCommand(0.0);
                }

                return;
            }


            double max_joint_vel = 0.0;
            for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
            {
                max_joint_vel = std::max(std::fabs(joint_vel(i)),max_joint_vel);
            }

            if(max_joint_vel > joint_vel_limit_)
            {
                double factor = joint_vel_limit_ / max_joint_vel;
                for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
                {
                    joint_vel(i) = joint_vel(i) * factor;
                }
            }

            // Convert the wrench into joint efforts
            for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
            {
                joint_handles_[i].setCommand(joint_vel(i));
            }


        }
        else
        {
            for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
            {
                joint_handles_[i].setCommand(0.0);
            }

        }

    }

    void CartesianController::starting(const ros::Time& time)
    {
        cmd_linear_twist_[0] = 0.0;
        cmd_linear_twist_[1] = 0.0;
        cmd_linear_twist_[2] = 0.0;
        cmd_linear_twist_[3] = 0.0;
        cmd_linear_twist_[4] = 0.0;
        cmd_linear_twist_[5] = 0.0;

        cmd_angular_twist_[0] = 0.0;
        cmd_angular_twist_[1] = 0.0;
        cmd_angular_twist_[2] = 0.0;
        cmd_angular_twist_[3] = 0.0;
        cmd_angular_twist_[4] = 0.0;
        cmd_angular_twist_[5] = 0.0;

        last_msg_ = ros::Time::now();
        got_msg_ = false;
    }

    void CartesianController::stopping(const ros::Time& time)
    {

    }

    void CartesianController::velCmdCB(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        cmd_linear_twist_.vel(0) = msg->twist.linear.x;
        cmd_linear_twist_.vel(1) = msg->twist.linear.y;
        cmd_linear_twist_.vel(2) = msg->twist.linear.z;
        cmd_linear_twist_.rot(0) = 0.0;
        cmd_linear_twist_.rot(1) = 0.0;
        cmd_linear_twist_.rot(2) = 0.0;

        cmd_angular_twist_.vel(0) = 0.0;
        cmd_angular_twist_.vel(1) = 0.0;
        cmd_angular_twist_.vel(2) = 0.0;
        cmd_angular_twist_.rot(0) = msg->twist.angular.x;
        cmd_angular_twist_.rot(1) = msg->twist.angular.y;
        cmd_angular_twist_.rot(2) = msg->twist.angular.z;

        last_msg_ = ros::Time::now();
        got_msg_ = true;

    }

}
