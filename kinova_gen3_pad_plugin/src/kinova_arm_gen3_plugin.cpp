#include <kinova_gen3_pad_plugin/kinova_arm_gen3_plugin.h>

namespace pad_plugins
{
PadPluginKinovaArmGen3::PadPluginKinovaArmGen3()
{
}

PadPluginKinovaArmGen3::~PadPluginKinovaArmGen3()
{
}

void PadPluginKinovaArmGen3::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = ros::NodeHandle();

  arm_control_topic_name_ = "/robot/arm/in/cartesian_velocity";
  readParam(pnh_, "arm_control_topic_name", arm_control_topic_name_, arm_control_topic_name_, required);
  gripper_topic_name_ = "/robot/arm/robotiq_2f_85_gripper_controller/gripper_cmd/goal";
  readParam(pnh_, "gripper_topic_name", gripper_topic_name_, gripper_topic_name_, required);
  axis_topic_name_ = "/robot/arm/in/joint_velocity";
  readParam(pnh_, "axis_topic_name", axis_topic_name_, axis_topic_name_, required);
  joint_state_topic_name_ = "/robot/arm/base_feedback/joint_state";
  readParam(pnh_, "joint_state_topic_name", joint_state_topic_name_, joint_state_topic_name_, required);

  set_home_service_name_ = "/robot/arm/in/home_arm";
  readParam(pnh_, "set_home_service_name", set_home_service_name_, set_home_service_name_, required);
  readParam(pnh_, "max_linear_speed", max_linear_speed_, 2.0, required);
  readParam(pnh_, "max_angular_speed", max_angular_speed_, 3.0, required);
  
  readParam(pnh_, "config/button_deadman", button_deadman_, 4, required);
  readParam(pnh_, "config/button_movement_deadman", button_movement_deadman_, 5, required);
  readParam(pnh_, "config/button_home_arm", button_home_arm_, 12, required);
  
  readParam(pnh_, "config/axis_open_gripper", axis_open_gripper_, 4, required);
  readParam(pnh_, "config/axis_close_gripper", axis_close_gripper_, 3, required);
  
  readParam(pnh_, "config/axis_linear_x_ee", axis_linear_x_, 1, required);
  readParam(pnh_, "config/axis_linear_y_ee", axis_linear_y_, 0, required);
  readParam(pnh_, "config/axis_linear_z_ee", axis_linear_z_, 2, required);

  //ROS_ERROR(" Using Linear  %d %d %d ",axis_linear_x_,axis_linear_y_,axis_linear_z_);

  readParam(pnh_, "config/axis_angular_x_ee", axis_angular_x_, 1, required);
  readParam(pnh_, "config/axis_angular_y_ee", axis_angular_y_, 0, required);
  readParam(pnh_, "config/axis_angular_z_ee", axis_angular_z_, 2, required);

  //ROS_ERROR(" Using Angular  %d %d %d ",axis_angular_x_,axis_angular_y_,axis_angular_z_);
  
  readParam(pnh_, "config/axis_speed", axis_speed_up_, 10, required);
  
  
  readParam(pnh_, "config/button_switch_mode", button_switch_mode_, 8, required);
  readParam(pnh_, "config/axis_inc_axis", axis_inc_axis_, 9, required);
  
  // Publishers
  arm_control_pub_ = nh_.advertise<kortex_driver::TwistCommand>(arm_control_topic_name_, 10);
  pad_status_pub_ = pnh_.advertise<kinova_gen3_pad_msgs::KinovaArmStatus>("status", 10);
  gripper_pub_ = pnh_.advertise<control_msgs::GripperCommandActionGoal>(gripper_topic_name_, 1);
  axis_pub_ = pnh_.advertise<kortex_driver::Base_JointSpeeds>(axis_topic_name_, 10);
  
  timerPublish = pnh_.createTimer(ros::Duration(0.2), &PadPluginKinovaArmGen3::timerPublishCallback, this);

  joint_state_sub_=pnh_.subscribe<sensor_msgs::JointState>(joint_state_topic_name_, 1, &PadPluginKinovaArmGen3::jointCB, this);

  // Services
  //set_home_service_ = nh_.serviceClient<kinova_msgs::HomeArm>(set_home_service_name_);

  // initialize variables
  current_velocity_level_ = 0.1;
  velocity_level_step_ = 0.1;
  max_velocity_level_ = 1.0;
  min_velocity_level_ = 0.1;
  fingers_closure_percentage_ = 0.0;
  arm_control_msg_ = kortex_driver::TwistCommand();
  arm_status_msg_ = kinova_gen3_pad_msgs::KinovaArmStatus();

  current_mode_ = 0;
  current_axis_ = 0;
  bPublish = false;
  bPublishGripper = false;
  current_gripper_position_ = -1.0;
  bFirstRead = false;
  bFirstSent = false;
  bIncUsed = false;
  bIncSpeedUsed = false;
}

void PadPluginKinovaArmGen3::timerPublishCallback(const ros::TimerEvent& event){
  
  bPublish = true;
  bPublishGripper = true;

}

void PadPluginKinovaArmGen3::jointCB(const sensor_msgs::JointState::ConstPtr& status_msg){  
  current_gripper_position_=status_msg->position[7];
  if (!bFirstRead){
    fingers_closure_percentage_=current_gripper_position_;
    fingers_closure_percentage_sent_=current_gripper_position_;
  }
  bFirstRead = true;
}

void PadPluginKinovaArmGen3::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_deadman_].isPressed() and not buttons[button_movement_deadman_].isPressed())
  {
    /*
    if (buttons[button_speed_down_].isReleased())
    {
      current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
      ROS_INFO("PadPluginKinovaArmGen3::execute: speed down -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }
    else if (buttons[button_speed_up_].isReleased())
    {
      current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
      ROS_INFO("PadPluginKinovaArmGen3::execute: speed up -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }*/

    if (axes[axis_speed_up_]<-0.9) {
        if (!bIncSpeedUsed) {
          current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
          ROS_INFO("PadPluginKinovaArmGen3::execute: speed down -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
          bIncSpeedUsed=true;
        }
      } else if (axes[axis_speed_up_]>0.9) {    
        if (!bIncUsed) {
          current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
          ROS_INFO("PadPluginKinovaArmGen3::execute: speed up -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
          bIncSpeedUsed=true;
        }
      } else if (axes[axis_speed_up_]==0.) {
        bIncSpeedUsed=false;
      }

    if (buttons[button_switch_mode_].isReleased()) {
      current_mode_++;
      ROS_INFO("mode");
    }
    if (current_mode_>2) current_mode_=0;
    if (current_mode_==2) {
      if (axes[axis_inc_axis_]>0.9) {
        ROS_INFO("axis");
        if (!bIncUsed) {
          current_axis_--;
          bIncUsed=true;
        }
      } else if (axes[axis_inc_axis_]<-0.9) {
        ROS_INFO("axis");        
        if (!bIncUsed) {
          current_axis_++;
          bIncUsed=true;
        }
      } else if (axes[axis_inc_axis_]==0.) {
        bIncUsed=false;
      }
      if (current_axis_>6) current_axis_=0;
      if (current_axis_<0) current_axis_=6;
    }

    
    if (bPublishGripper){
      //fingers_closure_percentage_= current_gripper_position_;
      if (axes[axis_open_gripper_]<-0.9) {
        fingers_closure_percentage_=fingers_closure_percentage_-0.04;
        ROS_INFO("open %f",axes[axis_open_gripper_]);
      }  else if (axes[axis_close_gripper_]<-0.9) {
        fingers_closure_percentage_=fingers_closure_percentage_+0.04;  
        ROS_INFO("close %f",axes[axis_close_gripper_]);
      } 
      if (fingers_closure_percentage_>0.8) fingers_closure_percentage_=0.8;
      if (fingers_closure_percentage_<0.0) fingers_closure_percentage_=0.0;
    
      GripperActionGoal_.goal.command.max_effort=0.1;
      GripperActionGoal_.goal.command.position=fingers_closure_percentage_;
      if (fabs(fingers_closure_percentage_sent_-fingers_closure_percentage_)>=0.001){
        gripper_pub_.publish(GripperActionGoal_);
        ROS_INFO("Publish");
        fingers_closure_percentage_sent_=fingers_closure_percentage_;
      }
      bPublishGripper=false;
    }
    
    if (current_mode_==1) {
      arm_control_msg_.twist.linear_x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_];
      arm_control_msg_.twist.linear_y = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_y_];
      arm_control_msg_.twist.linear_z = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_z_];
      arm_control_pub_.publish(arm_control_msg_);
    }

    if (current_mode_==0) {
      arm_control_msg_.twist.angular_x = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_x_];
      arm_control_msg_.twist.angular_y = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_y_];
      arm_control_msg_.twist.angular_z = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_z_];
      arm_control_pub_.publish(arm_control_msg_);
    }

    if (current_mode_==2) {
      
      kortex_driver::JointSpeed JS;

      JS.joint_identifier=current_axis_;
      JS.value = -axes[axis_linear_y_]*0.2;
      JS.duration = 1;

      axis_msg_.duration = 1;
      axis_msg_.joint_speeds.clear();
      axis_msg_.joint_speeds.push_back(JS);
      if (bPublish){
        axis_pub_.publish(axis_msg_);
        bPublish = false;
      }
    }
    
  }
  else {
    if (buttons[button_deadman_].isReleased())
    {
      arm_control_msg_.twist.linear_x = 0.0;
      arm_control_msg_.twist.linear_y = 0.0;
      arm_control_msg_.twist.linear_z = 0.0;
      arm_control_msg_.twist.angular_x = 0.0;
      arm_control_msg_.twist.angular_y = 0.0;
      arm_control_msg_.twist.angular_z = 0.0;
      //arm_control_msg_.fingers_closure_percentage = fingers_closure_percentage_;

      arm_control_pub_.publish(arm_control_msg_);
      GripperActionGoal_.goal.command.max_effort=0.1;
      GripperActionGoal_.goal.command.position=fingers_closure_percentage_;
      
      gripper_pub_.publish(GripperActionGoal_);
      ROS_INFO("Publish no dead");
      fingers_closure_percentage_sent_=fingers_closure_percentage_;
        
        
    }
    fingers_closure_percentage_=current_gripper_position_;
    fingers_closure_percentage_sent_=current_gripper_position_;
    if (bFirstRead){
      if (!bFirstSent){
        GripperActionGoal_.goal.command.max_effort=0.1;
        GripperActionGoal_.goal.command.position=fingers_closure_percentage_;
        //if (fabs(fingers_closure_percentage_sent_-fingers_closure_percentage_)>=0.001){
          gripper_pub_.publish(GripperActionGoal_);
          ROS_INFO("Publish sent");
          fingers_closure_percentage_sent_=fingers_closure_percentage_;
        //}
        bFirstSent=true;
      }
    }
  }
  arm_status_msg_.velocity_level = current_velocity_level_ * 100;
  arm_status_msg_.fingers_closure_percentage = fingers_closure_percentage_;
  arm_status_msg_.fingers_closure_percentage_read = current_gripper_position_;
  arm_status_msg_.current_axis = current_axis_;
  if (current_mode_==0) arm_status_msg_.current_mode = "Angular";
  if (current_mode_==1) arm_status_msg_.current_mode = "Linear";
  if (current_mode_==2) arm_status_msg_.current_mode = "Axis";
  //arm_status_msg_.fingers_closure_percentage = fingers_closure_percentage_;
  pad_status_pub_.publish(arm_status_msg_);

  //ROS_INFO("open %f close %f",axes[axis_open_gripper_],axes[axis_close_gripper_]);

}

}  // namespace pad_plugins
