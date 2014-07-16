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
#ifndef TEDUSAR_CARTESIAN_ARM_TELEOP__CARTESIAN_ARM_TELEOP_NODE_H_
#define TEDUSAR_CARTESIAN_ARM_TELEOP__CARTESIAN_ARM_TELEOP_NODE_H_

#include <string>
#include <boost/scoped_ptr.hpp>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>


namespace tedusar_cartesian_arm_teleop
{

class CartesianArmTeleopNode
{
public:
    CartesianArmTeleopNode();

    void init();

private:
    struct Parameters
    {
        double publication_rate_;
        std::string frame_id_;

        double std_translation_vel_;
        double std_rotation_vel_;
        double turbo_factor_;

        bool have_gripper_;
        double gripper_grasp_position_;
        double gripper_grasp_effort_;
        double gripper_release_position_;
        double gripper_release_effort_;

        std::string controller_;

        int button_translation_deadman_switch_;
        int button_rotation_deadman_switch_;
        int button_turbo_;
        int button_gripper_grasp_;
        int button_gripper_release_;
        int button_grasp_done_;

        int axis_translation_x_;
        int axis_translation_y_;
        int axis_translation_z_;
        int axis_rotation_x_;
        int axis_rotation_y_;
        int axis_rotation_z_;
    };

    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;

    void loadParameters();

    void cmdGeneratorTimerCB(const ros::TimerEvent & e);

    void joyCB(const sensor_msgs::Joy::ConstPtr & joy);

    void sendCmdVel();
    void sendGripperCommand(double position, double max_effort);

    void sendGraspDone();

    template <typename T> static void getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, T & value);
    template <typename T> static void getOptionalParameter(ros::NodeHandle & private_nh, const std::string & key, T & value, T default_value);

    static double limit(double value, double min, double max);

    ros::NodeHandle nh_;
    Parameters parameters_;
    bool repeat_vel_commands_;

    ros::Subscriber joy_sub_;
    ros::Timer cmd_generator_timer_;
    ros::Publisher cmd_vel_pub_;
    boost::scoped_ptr<GripperCommandActionClient> gripper_command_ac_;

    ros::Publisher grasping_done_pub_;

    geometry_msgs::TwistStamped cmd_vel_;
};

}

#endif // TEDUSAR_CARTESIAN_ARM_TELEOP__CARTESIAN_ARM_TELEOP_NODE_H_
