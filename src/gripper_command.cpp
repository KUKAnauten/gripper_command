  /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 *********************************************************************/

/* Author: Niklas Schaefer */

#include <robotiq_s_model_control/s_model_msg_client.h>
#include <robotiq_s_model_control/s_model_api.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

using namespace robotiq;

namespace gripper_command {

class GripperCommand {  
  private:
    boost::shared_ptr<robotiq_s_model_control::SModelMsgClient> sModelMsgClient_;
    ros::Subscriber gripper_command_sub_;
    ros::NodeHandle* node_handle_;
    int toggle_previous_;


  void toggleCallback(const std_msgs::Float64::ConstPtr& msg) {
    double command = msg->data;

    if(int(command) != toggle_previous_) {
      if(int(command) == 0) {
        openGripper();
      }
      else if(int(command) == 1) {
        closeGripper();
      }
    }

    toggle_previous_ = int(command);
  }

  void positionCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    uint8_t position_desired = msg->data;
    gripper.setRawPosition(position_desired);
    gripper.write();  
  }

  public:
    robotiq_s_model_control::SModelAPI gripper;

    GripperCommand(ros::NodeHandle* node_handle)
        : sModelMsgClient_(new robotiq_s_model_control::SModelMsgClient(*node_handle)),
          gripper(sModelMsgClient_),
          node_handle_(node_handle),
          toggle_previous_(0) {
    }

    void registerSubscriber(const std::string& topic) {
      gripper_command_sub_ = node_handle_->subscribe(topic, 1, &GripperCommand::positionCallback, this);
    }

    // void initializeGripper(int position, int velocity, int force) {

    // }

    void openGripper() {
      gripper.setRawPosition(0);
      gripper.write();    
    }

    void closeGripper() {
      gripper.setRawPosition(255);
      gripper.write();    
    }

    // void rawPositionCommand(const unsigned char& rawPosition ) {
    //   gripper.setRawPosition(rawPosition);
    //   gripper.write();
    // }

    void waitForGripper() {
      ros::Duration(0.1).sleep();
      do {
        //if (!gripper.isInitialized()) std::cout <<  "Not initialized!" << std::endl;
        //if (!gripper.isReady()) std::cout << "Not ready!" << std::endl;
        //if (gripper.isMoving()) std::cout << "Still Moving!" << std::endl;
        gripper.read();
        ros::Duration(0.1).sleep();
      } while (ros::ok() && (!gripper.isInitialized() || !gripper.isReady() || gripper.isMoving()));
      ros::Duration(0.1).sleep();
    }

  };
} // namespace gripper_command

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_gripper_command");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  gripper_command::GripperCommand gripper_commander(&node_handle);

  gripper_commander.gripper.setInitialization(INIT_ACTIVATION);
  gripper_commander.gripper.setGraspingMode(GRASP_PINCH);
  gripper_commander.gripper.setActionMode(ACTION_GO);
  // Setting "raw" values [0, 255] instead of mm/s for velocity, N for force, etc.
  gripper_commander.gripper.setRawVelocity(255);
  gripper_commander.gripper.setRawForce(50);

  gripper_commander.gripper.setRawPosition(0);
  gripper_commander.gripper.write();
  gripper_commander.waitForGripper();

  gripper_commander.registerSubscriber(std::string("gripper_in_position"));
  ROS_INFO_NAMED("gripper_commander", "Subscribed to gripper_in_position!");

  ros::Rate rate(100);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
