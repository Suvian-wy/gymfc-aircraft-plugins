/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_servo_model.h"
#include "EscSensor.pb.h"
#include <ignition/math.hh>

#define DEG2RAD 0.017453292
#define SERVO_ACCURACY 0.001526112

namespace gazebo {

GazeboServoModel::~GazeboServoModel() { updateConnection_->~Connection(); }

void GazeboServoModel::InitializeParams() { }

void GazeboServoModel::Publish()
{
    if (rotor_velocity_units_.compare(rotor_velocity_units::RPM) == 0) {
        double rpms = std::abs(joint_->GetVelocity(0) * 9.5493);
        // gzdbg << "Publishing ESC sensors " << motor_number_ << " Velocity " << rpms << std::endl;
        sensor.set_motor_speed(rpms);
    } else {
        sensor.set_motor_speed(std::abs(joint_->GetVelocity(0)));
    }

    // XXX When id was of type int64 motor speed will be reset
    // after id is set.
    sensor.set_id(motor_number_);

    // TODO develope model for these
    sensor.set_current(0);
    sensor.set_temperature(0);
    sensor.set_voltage(0);

    sensor.set_force(0);
    sensor.set_torque(0);

    esc_sensor_pub_->Publish(sensor);
    // gzdbg << "Sending esc sensor for motor " << motor_number_ << std::endl;
}

void GazeboServoModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;

    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("jointName"))
        joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
    // Get the pointer to the joint.
    joint_ = model_->GetJoint(joint_name_);
    if (joint_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");

    if (_sdf->HasElement("linkName"))
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");

    if (_sdf->HasElement("motorNumber"))
        motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
    else
        gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

    if (_sdf->HasElement("servoDynamicAngle")) {
        sdf::ElementPtr tf = _sdf->GetElement("servoDynamicAngle");
        for (int i = 0; i < 4; i++) {
            servo_dynamic_angle_[i] = 0;
        }
        if (tf->HasElement("delayOneTerm")) {
            servo_dynamic_angle_[0] = tf->Get<double>("delayOneTerm");
        }
        if (tf->HasElement("delayTwoTerm")) {
            servo_dynamic_angle_[1] = tf->Get<double>("delayTwoTerm");
        }
        if (tf->HasElement("inputCurTerm")) {
            servo_dynamic_angle_[2] = tf->Get<double>("inputCurTerm");
        }
        if (tf->HasElement("inputLastTerm")) {
            servo_dynamic_angle_[3] = tf->Get<double>("inputLastTerm");
        }

    } else {
        servo_dynamic_angle_[0] = 1.906;
        servo_dynamic_angle_[1] = -0.909091;
        servo_dynamic_angle_[2] = 0.001361;
        servo_dynamic_angle_[3] = 0.001318;
    }

    getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
    getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);

    getSdfParam<double>(_sdf, "servoInitPwm", servo_init_pwm_, servo_init_pwm_);
    getSdfParam<double>(_sdf, "servoMinPwm", servo_min_pwn_, servo_min_pwn_);
    getSdfParam<double>(_sdf, "servoMaxPwm", servo_max_pwn_, servo_max_pwn_);
    getSdfParam<double>(_sdf, "servoPwmRange", servo_pwm_range_, servo_pwm_range_);
    getSdfParam<double>(_sdf, "servoWb", servo_wb_, servo_wb_);
    getSdfParam<double>(_sdf, "servoCr", servo_cr_, servo_cr_);
    getSdfParam<double>(_sdf, "servoMaxRad", servo_max_rad_, servo_max_rad_);

    // getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

    getSdfParam<std::string>(_sdf, "rotorVelocityUnits", rotor_velocity_units_, rotor_velocity_units_);

    // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
    joint_->SetMaxForce(0, max_force_);
#endif
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboServoModel::OnUpdate, this, _1));

    command_sub_ = node_handle_->Subscribe<cmd_msgs::msgs::MotorCommand>(
        command_sub_topic_, &GazeboServoModel::VelocityCallback, this);
    // std::cout << "[gazebo_motor_model]: Subscribe to gz topic: "<< motor_failure_sub_topic_ << std::endl;
    motor_failure_sub_
        = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &GazeboServoModel::MotorFailureCallback, this);

    esc_sensor_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EscSensor>(
        motor_speed_pub_topic_ + "/" + std::to_string(motor_number_));

    // // Create the first order filter.
    // rotor_velocity_filter_.reset(
    //     new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));

    gzdbg << "Loading Motor number=" << motor_number_ << " Subscribed to " << command_sub_topic_ << std::endl;
}

// This gets called by the world update start event.
void GazeboServoModel::OnUpdate(const common::UpdateInfo& _info)
{
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    prev_sim_time_ = _info.simTime.Double();
    UpdateForcesAndMoments();
    UpdateMotorFail();
    Publish();
}
void GazeboServoModel::Reset()
{
    // gzdbg << "Resetting motor " << motor_number_ << std::endl;
    joint_->Reset();
    joint_->SetVelocity(0, 0);
    prev_sim_time_      = 0;
    sampling_time_      = 0.01;
    ref_servo_rad_      = 0;
    last_ref_servo_rad_ = 0;
    cur_servo_rad_      = 0;
    last_servo_rad_     = 0;
    // joint_->SetPosition(0, 90 * DEG2RAD);
}
// TODO:PWM到舵机角度的映射关系
double GazeboServoModel::CommandTransferFunction(double x)
{
    // gzerr << "the cmd x is: " <<((x * servo_pwm_range_ / 2 + servo_init_pwm_) * servo_cr_ + servo_wb_) << "\n";
    return (x * servo_pwm_range_ / 2 + servo_init_pwm_) * servo_cr_ + servo_wb_;
}

void GazeboServoModel::VelocityCallback(MotorCommandPtr& _cmd)
{
    // gzdbg << "Motor " << motor_number_ << " Value " << _cmd->motor(motor_number_) << " Current velocity " <<
    // joint_->GetVelocity(0) << std::endl;

    if (_cmd->motor_size() < motor_number_) {
        std::cout << "You tried to access index " << motor_number_
                  << " of the MotorSpeed message array which is of size " << _cmd->motor_size() << "." << std::endl;
    } else {
        last_ref_servo_rad_ = ref_servo_rad_;
        ref_servo_rad_      = CommandTransferFunction(static_cast<double>(_cmd->motor(motor_number_)));
        ref_servo_rad_      = (abs(ref_servo_rad_) > SERVO_ACCURACY) ? ref_servo_rad_ : 0;
    }
}

void GazeboServoModel::MotorFailureCallback(const boost::shared_ptr<const msgs::Int>& fail_msg)
{
    motor_Failure_Number_ = fail_msg->data();
}

void GazeboServoModel::UpdateForcesAndMoments()
{

    // joint_->SetPosition(0, 90 * DEG2RAD);

    cur_servo_rad_ = joint_->Position() / DEG2RAD;
    cur_servo_rad_ = (abs(cur_servo_rad_) > SERVO_ACCURACY) ? cur_servo_rad_ : 0;

    double tmp_rad = cur_servo_rad_;

    cur_servo_rad_ = servo_dynamic_angle_[0] * cur_servo_rad_ + servo_dynamic_angle_[1] * last_servo_rad_
                     + servo_dynamic_angle_[2] * ref_servo_rad_ + servo_dynamic_angle_[3] * last_ref_servo_rad_;
    last_servo_rad_ = tmp_rad;

    joint_->SetPosition(0, cur_servo_rad_ * DEG2RAD);
    // joint_->SetUpperLimit(0, cur_servo_rad_ * DEG2RAD);
    // joint_->SetLowerLimit(0, cur_servo_rad_ * DEG2RAD);
    double position = joint_->Position() / DEG2RAD;
    // gzerr << "the position is\t" << position << "," << cur_servo_rad_ << "," << ref_servo_rad_ << "\n";
}

void GazeboServoModel::UpdateMotorFail()
{
    if (motor_number_ == motor_Failure_Number_ - 1) {
        // motor_constant_ = 0.0;
        joint_->SetVelocity(0, 0);
        if (screen_msg_flag) {
            std::cout << "Motor number [" << motor_Failure_Number_ << "] failed!  [Motor thrust = 0]" << std::endl;
            tmp_motor_num = motor_Failure_Number_;

            screen_msg_flag = 0;
        }
    } else if (motor_Failure_Number_ == 0 && motor_number_ == tmp_motor_num - 1) {
        if (!screen_msg_flag) {
            // motor_constant_ = kDefaultMotorConstant;
            std::cout << "Motor number [" << tmp_motor_num << "] running! [Motor thrust = (default)]" << std::endl;
            screen_msg_flag = 1;
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboServoModel);
}
