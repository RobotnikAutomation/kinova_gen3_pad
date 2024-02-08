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
  arm_base_joint_control_topic_name_ = "/robot/j2s6s200_driver/in/joint_velocity";
  readParam(pnh_, "arm_base_control_topic_name", arm_base_joint_control_topic_name_, arm_base_joint_control_topic_name_, required);
  arm_stop_motion_topic_name_ = "/my_gen3/in/stop";
  readParam(pnh_, "arm_stop_motion_topic_name", arm_stop_motion_topic_name_, arm_stop_motion_topic_name_, required);
  set_home_service_name_ = "/robot/arm/in/home_arm";
  readParam(pnh_, "set_home_service_name", set_home_service_name_, set_home_service_name_, required);
  gripper_command_service_name_ = "/my_gen3/base/send_gripper_command";
  readParam(pnh_, "gripper_command_service_name", gripper_command_service_name_, gripper_command_service_name_, required);
  readParam(pnh_, "max_linear_speed", max_linear_speed_, 2.0, required);
  readParam(pnh_, "max_angular_speed", max_angular_speed_, 3.0, required);
  readParam(pnh_, "max_base_speed", max_base_speed_, 0.5, required);
  readParam(pnh_, "config/button_deadman", button_deadman_, 4, required);
  readParam(pnh_, "config/button_movement_deadman", button_movement_deadman_, 5, required);
  readParam(pnh_, "config/button_home_arm", button_home_arm_, 12, required);
  readParam(pnh_, "config/axis_open_gripper", axis_open_gripper_, 4, required);
  readParam(pnh_, "config/axis_close_gripper", axis_close_gripper_, 3, required);
  readParam(pnh_, "config/axis_linear_x_ee", axis_linear_x_, 1, required);
  readParam(pnh_, "config/axis_linear_y_ee", axis_linear_y_, 0, required);
  readParam(pnh_, "config/axis_linear_z_ee", axis_linear_z_, 10, required);
  readParam(pnh_, "config/axis_angular_x_ee", axis_angular_x_, 5, required);
  readParam(pnh_, "config/axis_angular_y_ee", axis_angular_y_, 2, required);
  readParam(pnh_, "config/axis_angular_z_ee", axis_angular_z_, 9, required);
  readParam(pnh_, "config/button_speed_up", button_speed_up_, 3, required);
  readParam(pnh_, "config/button_speed_down", button_speed_down_, 1, required);
  readParam(pnh_, "config/button_rotate_base_pos", button_rot_base_pos_, 9, required);
  readParam(pnh_, "config/button_rotate_base_neg", button_rot_base_neg_, 10, required);
  readParam(pnh_, "config/button_open_gripper", button_open_gripper_, 0, required);
  readParam(pnh_, "config/button_close_gripper", button_close_gripper_, 2, required);

  // Publishers
  arm_control_pub_ = nh_.advertise<kortex_driver::TwistCommand>(arm_control_topic_name_, 10);
  arm_base_joint_control_pub_ = nh_.advertise<kortex_driver::Base_JointSpeeds>(arm_base_joint_control_topic_name_, 10);
  pad_status_pub_ = pnh_.advertise<kinova_gen3_pad_msgs::KinovaArmStatus>("status", 10);
  stop_motion_pub_ = pnh_.advertise<std_msgs::Empty>(arm_stop_motion_topic_name_, 1);

  // Services
  //set_home_service_ = nh_.serviceClient<kinova_msgs::HomeArm>(set_home_service_name_);
  gripper_command_client_ = nh_.serviceClient<kortex_driver::SendGripperCommand>(gripper_command_service_name_);
  gripper_command_client_.waitForExistence();
  
  // initialize variables
  gripper_pressed_flag_ = false;
  current_velocity_level_ = 0.1;
  velocity_level_step_ = 0.1;
  max_velocity_level_ = 1.0;
  min_velocity_level_ = 0.1;
  fingers_closure_percentage_ = 0.0;
  arm_control_msg_ = kortex_driver::TwistCommand();
  arm_status_msg_ = kinova_gen3_pad_msgs::KinovaArmStatus();
  arm_joint_control_msg_ = kortex_driver::Base_JointSpeeds();
  arm_joint_control_msg_.joint_speeds.resize(1);
  arm_base_joint_control_msg_ = kortex_driver::JointSpeed();
  arm_base_joint_control_msg_.joint_identifier = 0;
  arm_joint_control_msg_.joint_speeds[0] = arm_base_joint_control_msg_;
  gripper_command_ = kortex_driver::SendGripperCommand();
  gripper_command_.request.input.mode = 2;
  base_frame_ = "robot_arm_base_link";
  tool_frame_ = "robot_arm_end_effector_link";
  stop_motion_ = std_msgs::Empty();
}

void PadPluginKinovaArmGen3::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_deadman_].isPressed() and not buttons[button_movement_deadman_].isPressed())
  {
    if (buttons[button_speed_down_].isReleased())
    {
      current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
      ROS_INFO("PadPluginKinovaArmGen3::execute: speed down -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }
    else if (buttons[button_speed_up_].isReleased())
    {
      current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
      ROS_INFO("PadPluginKinovaArmGen3::execute: speed up -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }

    if (buttons[button_rot_base_pos_].isPressed())
    {
       arm_joint_control_msg_.joint_speeds[0].value = max_base_speed_;
    }

    if (buttons[button_rot_base_neg_].isPressed())
    {
       arm_joint_control_msg_.joint_speeds[0].value = -max_base_speed_;
    }

    if (buttons[button_rot_base_pos_].isReleased())
    {
       stop_motion_pub_.publish(stop_motion_);
       arm_joint_control_msg_.joint_speeds[0].value = 0.0;
    }

    if (buttons[button_rot_base_neg_].isReleased())
    {
       stop_motion_pub_.publish(stop_motion_);
       arm_joint_control_msg_.joint_speeds[0].value = 0.0;
    }

    if (buttons[button_open_gripper_].isPressed())
    {
      if(gripper_pressed_flag_ == false)
      { 
        gripper_pressed_flag_= true;
        kortex_driver::Finger finger;
        finger.finger_identifier = 1;
        finger.value = 0.5;
        gripper_command_.request.input.gripper.finger.clear();
        gripper_command_.request.input.gripper.finger.push_back(finger);
        gripper_command_client_.call(gripper_command_);
      }
    }

    if (buttons[button_open_gripper_].isReleased())
    {
        gripper_pressed_flag_= false;
        kortex_driver::Finger finger;
        finger.finger_identifier = 1;
        finger.value = 0.0;
        gripper_command_.request.input.gripper.finger.clear();
        gripper_command_.request.input.gripper.finger.push_back(finger);
        gripper_command_client_.call(gripper_command_);
    }

    if (buttons[button_close_gripper_].isPressed())
    {
      if(gripper_pressed_flag_ == false)
      { 
        gripper_pressed_flag_= true;
        kortex_driver::Finger finger;
        finger.finger_identifier = 1;
        finger.value = -0.5;
        gripper_command_.request.input.gripper.finger.clear();
        gripper_command_.request.input.gripper.finger.push_back(finger);
        gripper_command_client_.call(gripper_command_);
      }
    }

    if (buttons[button_close_gripper_].isReleased())
    {
        gripper_pressed_flag_= false;
        kortex_driver::Finger finger;
        finger.finger_identifier = 1;
        finger.value = 0.0;
        gripper_command_.request.input.gripper.finger.clear();
        gripper_command_.request.input.gripper.finger.push_back(finger);
        gripper_command_client_.call(gripper_command_);
    }




   try {
        // Get the transform from the tool frame to the base frame
        tf_listener_.waitForTransform(base_frame_, tool_frame_, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(base_frame_, tool_frame_, ros::Time(0), transform_);


        if(std::abs(axes[axis_linear_x_]) <= 0.3)
        { 
          axes[axis_linear_x_] = 0;

        }
        if(std::abs(axes[axis_linear_y_]) <= 0.3)
        {
          axes[axis_linear_y_] = 0;
        }     
        if(std::abs(axes[axis_linear_z_]) <= 0.3)
        {
          axes[axis_linear_z_] = 0;
        }    

        // Extract the rotation matrix from the transform
        Eigen::Matrix3d rotation_matrix;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rotation_matrix(i, j) = transform_.getBasis()[i][j];

        // Transform the linear velocities to the base frame using matrix multiplication
        Eigen::Vector3d linear_velocities( axes[axis_linear_x_], axes[axis_linear_y_], axes[axis_linear_z_]);
        Eigen::Vector3d rotated_linear_velocities = rotation_matrix * linear_velocities;

        // Create a new TwistStamped message with the transformed velocities
        transformed_twist_.twist.linear.x = rotated_linear_velocities.x();
        transformed_twist_.twist.linear.y = rotated_linear_velocities.y();
        transformed_twist_.twist.linear.z = rotated_linear_velocities.z();

    } catch (tf::TransformException& ex) {
        ROS_WARN("TF transform lookup failed: %s", ex.what());
    }


    arm_control_msg_.twist.linear_x = current_velocity_level_ * max_linear_speed_ * transformed_twist_.twist.linear.x;
    arm_control_msg_.twist.linear_y = current_velocity_level_ * max_linear_speed_ * transformed_twist_.twist.linear.y;
    arm_control_msg_.twist.linear_z = current_velocity_level_ * max_linear_speed_ * transformed_twist_.twist.linear.z;

    arm_control_msg_.twist.angular_x = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_x_];
    arm_control_msg_.twist.angular_y = - current_velocity_level_ * max_angular_speed_ * axes[axis_angular_y_];
    arm_control_msg_.twist.angular_z = - current_velocity_level_ * max_angular_speed_ * axes[axis_angular_z_] * 2;



    if( arm_joint_control_msg_.joint_speeds[0].value != 0)
    {
      arm_base_joint_control_pub_.publish(arm_joint_control_msg_);
      arm_joint_control_msg_.joint_speeds[0].value = 0.0;
    }else{
      arm_control_pub_.publish(arm_control_msg_);      
      arm_joint_control_msg_.joint_speeds[0].value = 0.0;
    }
  }
  else if (buttons[button_deadman_].isReleased())
  {
    stop_motion_pub_.publish(stop_motion_);

    arm_control_msg_.twist.linear_x = 0.0;
    arm_control_msg_.twist.linear_y = 0.0;
    arm_control_msg_.twist.linear_z = 0.0;
    arm_control_msg_.twist.angular_x = 0.0;
    arm_control_msg_.twist.angular_y = 0.0;
    arm_control_msg_.twist.angular_z = 0.0;
    //arm_control_msg_.fingers_closure_percentage = fingers_closure_percentage_;

    arm_control_pub_.publish(arm_control_msg_);

    arm_joint_control_msg_.joint_speeds[0].value = 0.0;

    arm_base_joint_control_pub_.publish(arm_joint_control_msg_);
  }

  arm_status_msg_.velocity_level = current_velocity_level_ * 100;
  //arm_status_msg_.fingers_closure_percentage = fingers_closure_percentage_;
  pad_status_pub_.publish(arm_status_msg_);
}

}  // namespace pad_plugins
