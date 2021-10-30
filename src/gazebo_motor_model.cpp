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

#include "gazebo_motor_model.h"
#include "EscSensor.pb.h"
#include <ignition/math.hh>

namespace gazebo {

static int counter;

GazeboMotorModel::~GazeboMotorModel()
{
    updateConnection_->~Connection();
    use_pid_ = false;
}

void GazeboMotorModel::InitializeParams() { }

void GazeboMotorModel::Publish()
{
    if (rotor_velocity_units_.compare(rotor_velocity_units::RPM) == 0) {
        double rpms = std::abs(joint1_->GetVelocity(0) * 9.5493);
        // gzdbg << "Publishing ESC sensors " << motor_number_ << " Velocity " << rpms << std::endl;
        sensor.set_motor_speed(rpms);
    } else {
        sensor.set_motor_speed(std::abs(joint1_->GetVelocity(0)));
    }

    // XXX When id was of type int64 motor speed will be reset
    // after id is set.
    sensor.set_id(motor_number_);

    // TODO develope model for these
    sensor.set_current(0);
    sensor.set_temperature(0);
    sensor.set_voltage(0);

    sensor.set_force(current_force_);
    sensor.set_torque(current_torque_);

    esc_sensor_pub_->Publish(sensor);
    // gzdbg << "Sending esc sensor for motor " << motor_number_ << std::endl;
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;

    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("joint1Name"))
        joint1_name_ = _sdf->GetElement("joint1Name")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a joint1Name, where the rotor is attached.\n";
    // Get the pointer to the joint.
    joint1_ = model_->GetJoint(joint1_name_);
    if (joint1_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint1_name_ << "\".");

    if (_sdf->HasElement("joint2Name"))
        joint2_name_ = _sdf->GetElement("joint2Name")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a joint2Name, where the rotor is attached.\n";
    // Get the pointer to the joint.
    joint2_ = model_->GetJoint(joint2_name_);
    if (joint2_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint2_name_ << "\".");

    if (_sdf->HasElement("link1Name"))
        link1_name_ = _sdf->GetElement("link1Name")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a link1Name of the rotor.\n";
    link1_ = model_->GetLink(link1_name_);
    if (link1_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link1_name_ << "\".");

    if (_sdf->HasElement("link2Name"))
        link2_name_ = _sdf->GetElement("link2Name")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a link2Name of the rotor.\n";
    link2_ = model_->GetLink(link2_name_);
    if (link2_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link2_name_ << "\".");

    if (_sdf->HasElement("baseLinkName"))
        baselink_name_ = _sdf->GetElement("baseLinkName")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a baseLinkName of the rotor.\n";
    baselink_ = model_->GetLink(baselink_name_);
    if (baselink_ == NULL)
        gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << baselink_name_ << "\".");

    if (_sdf->HasElement("motorNumber"))
        motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
    else
        gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

    if (_sdf->HasElement("motorDynamicFlapHz")) {
        sdf::ElementPtr tf = _sdf->GetElement("motorDynamicFlapHz");
        for (int i = 0; i < 2; i++) {
            motor_dynamic_flap_hz_[i] = 0;
        }
        if (tf->HasElement("delayOneTerm")) {
            motor_dynamic_flap_hz_[0] = tf->Get<double>("delayOneTerm");
        }
        if (tf->HasElement("inputCurTerm")) {
            motor_dynamic_flap_hz_[1] = tf->Get<double>("inputCurTerm");
        }

    } else {
        motor_dynamic_flap_hz_[0] = 0.9416195857;
        motor_dynamic_flap_hz_[1] = 0.0660980810;
    }

    if (_sdf->HasElement("flapHzToThrust")) {
        sdf::ElementPtr tf = _sdf->GetElement("flapHzToThrust");
        for (int i = 0; i < 3; i++) {
            flap_hz_to_thrust_[i] = 0;
        }
        if (tf->HasElement("inputCurTerm")) {
            flap_hz_to_thrust_[0] = tf->Get<double>("inputCurTerm");
        }
        if (tf->HasElement("squareTerm")) {
            flap_hz_to_thrust_[1] = tf->Get<double>("squareTerm");
        }
        if (tf->HasElement("thrustNoiseCoef")) {
            flap_hz_to_thrust_[2] = tf->Get<double>("thrustNoiseCoef");
        }
    } else {
        flap_hz_to_thrust_[0] = 0.9416195857;
        flap_hz_to_thrust_[1] = 0.0660980810;
        flap_hz_to_thrust_[2] = 1;
    }

    if (_sdf->HasElement("copPosition")) {
        sdf::ElementPtr tf = _sdf->GetElement("copPosition");
        for (int i = 0; i < 3; i++) {
            cop_poistion_[i] = 0;
        }
        if (tf->HasElement("x")) {
            cop_poistion_[0] = tf->Get<double>("x");
        }
        if (tf->HasElement("y")) {
            cop_poistion_[1] = tf->Get<double>("y");
        }
        if (tf->HasElement("z")) {
            cop_poistion_[2] = tf->Get<double>("z");
        }
    } else {
        cop_poistion_[0] = 0.0;
        cop_poistion_[1] = 0.0;
        cop_poistion_[2] = 0.06;
    }

    if (_sdf->HasElement("dragCoefficient")) {
        sdf::ElementPtr tf = _sdf->GetElement("dragCoefficient");
        for (int i = 0; i < 3; i++) {
            air_drag_coefficient_[i] = 0;
        }
        if (tf->HasElement("cdx")) {
            air_drag_coefficient_[0] = tf->Get<double>("cdx");
        }
        if (tf->HasElement("cdy")) {
            air_drag_coefficient_[1] = tf->Get<double>("cdy");
        }
        if (tf->HasElement("cdz")) {
            air_drag_coefficient_[2] = tf->Get<double>("cdz");
        }

    } else {
        air_drag_coefficient_[0] = 0.0042;
        air_drag_coefficient_[1] = 0.0042;
        air_drag_coefficient_[2] = 0.0009;
    }

    if (_sdf->HasElement("turningDirection")) {
        std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
        if (turning_direction == "cw")
            turning_direction_ = turning_direction::CW;
        else if (turning_direction == "ccw")
            turning_direction_ = turning_direction::CCW;
        else
            gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
    } else
        gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

    getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
    getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);

    getSdfParam<double>(_sdf, "flapMaxRad", flap_max_rad_, flap_max_rad_);
    getSdfParam<double>(_sdf, "motorCr", motor_cr_, motor_cr_);
    getSdfParam<double>(_sdf, "motorWb", motor_wb_, motor_wb_);
    getSdfParam<double>(_sdf, "motorT", motor_T_, motor_T_);
    getSdfParam<double>(_sdf, "motorJm", motor_jm_, motor_jm_);
    getSdfParam<double>(_sdf, "initRPM", init_RPM_, init_RPM_);
    getSdfParam<double>(_sdf, "maxFlapHz", max_flap_hz, max_flap_hz);
    // getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

    getSdfParam<std::string>(_sdf, "rotorVelocityUnits", rotor_velocity_units_, rotor_velocity_units_);

    // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
    joint_->SetMaxForce(0, max_force_);
#endif
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

    command_sub_ = node_handle_->Subscribe<cmd_msgs::msgs::MotorCommand>(
        command_sub_topic_, &GazeboMotorModel::VelocityCallback, this);
    // std::cout << "[gazebo_motor_model]: Subscribe to gz topic: "<< motor_failure_sub_topic_ << std::endl;
    motor_failure_sub_
        = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &GazeboMotorModel::MotorFailureCallback, this);

    esc_sensor_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EscSensor>(
        motor_speed_pub_topic_ + "/" + std::to_string(motor_number_));

    // Create the first order filter.
    // rotor_velocity_filter_.reset(
    //     new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_flap_hz));

    gzdbg << "Loading Motor number=" << motor_number_ << " Subscribed to " << command_sub_topic_ << std::endl;
}

// Protobuf test
/*
void GazeboMotorModel::testProto(MotorSpeedPtr &msg) {
  std::cout << "Received message" << std::endl;
  std::cout << msg->motor_speed_size()<< std::endl;
  for(int i; i < msg->motor_speed_size(); i++){
    std::cout << msg->motor_speed(i) <<" ";
  }
  std::cout << std::endl;
}
*/

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info)
{
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    prev_sim_time_ = _info.simTime.Double();
    UpdateForcesAndMoments();
    UpdateMotorFail();
    Publish();
}
void GazeboMotorModel::Reset()
{
    // gzdbg << "Resetting motor " << motor_number_ << std::endl;
    joint1_->Reset();
    joint1_->SetVelocity(0, 0);
    joint2_->Reset();
    joint2_->SetVelocity(0, 0);
    motor_rot_vel_ = 0;
    ref_flap_hz    = 0;
    cur_flap_hz    = 0;
    prev_sim_time_ = 0;
    sampling_time_ = 0.001;
    turned_rad     = 0;
    counter        = 0;

    // rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_flap_hz));
}
double GazeboMotorModel::CommandTransferFunction(double x) { return 4.2 * x * motor_cr_ + motor_wb_; }

void GazeboMotorModel::VelocityCallback(MotorCommandPtr& _cmd)
{
    // gzdbg << "Motor " << motor_number_ << " Value " << _cmd->motor(motor_number_) << " Current velocity " <<
    // joint_->GetVelocity(0) << std::endl;

    if (_cmd->motor_size() < motor_number_) {
        std::cout << "You tried to access index " << motor_number_
                  << " of the MotorSpeed message array which is of size " << _cmd->motor_size() << "." << std::endl;
    } else
        ref_flap_hz = CommandTransferFunction(static_cast<double>(_cmd->motor(motor_number_)));
}

void GazeboMotorModel::MotorFailureCallback(const boost::shared_ptr<const msgs::Int>& fail_msg)
{
    motor_Failure_Number_ = fail_msg->data();
}

void GazeboMotorModel::UpdateForcesAndMoments()
{
    cur_flap_hz = 1 / (1 + (1 / motor_T_) * sampling_time_) * cur_flap_hz
                  + (1 / motor_T_) * sampling_time_ / (1 + (1 / motor_T_) * sampling_time_) * ref_flap_hz;
    current_force_ = (flap_hz_to_thrust_[0] * cur_flap_hz + flap_hz_to_thrust_[1] * cur_flap_hz * cur_flap_hz) / 9.8;
    double force   = current_force_;
    motor_rot_vel_ = joint1_->GetVelocity(0);
    // turned_rad += motor_rot_vel_ * sampling_time_;
    if (turned_rad >= flap_max_rad_ || turned_rad <= 0)
        turning_direction_ = turning_direction_ * -1;

    double real_motor_velocity = motor_rot_vel_ = cur_flap_hz * flap_max_rad_ * 2;

#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d body_velocity_1 = link1_->RelativeLinearVel();
    ignition::math::Vector3d body_velocity_2 = link2_->RelativeLinearVel();
    // ignition::math::Vector3d body_velocity   = body_velocity_2;
    ignition::math::Vector3d body_velocity = (body_velocity_1 + body_velocity_2) / 2;

    // ignition::math::Vector3d err_velocity = body_velocity_1 - body_velocity_2;
    // gzdbg << "the worldlinear vel is : " << err_velocity[0] << ", " << err_velocity[1] << ", " << err_velocity[2]
    //       << "\n";
    // gzdbg << "the linear vel is : " << body_velocity_1[0] << ", " << body_velocity_1[1] << ", "
    //       << body_velocity_1[2] << "\n";
#else
    ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link1_->GetWorldLinearVel());
#endif

    // double vel = body_velocity.Length();

    // double scalar = 1 - vel / 25;
    // force         = force * ignition::math::clamp(scalar, 0.0, 1.0);

    // gzdbg << "the z velocity is:  " << vel << "    force :  " << force << "\n";

    // Apply a force to the link.

    // ignition::math::Vector3d joint_axis          = joint1_->GlobalAxis(0);
    // ignition::math::Vector3d body_velocity1_flap = body_velocity_1 - (body_velocity_1 * joint_axis) * joint_axis;

    ignition::math::Vector3d air_drag1(0, 0, 0);
    air_drag1[0] = -std::abs(cur_flap_hz) * air_drag_coefficient_[0] * body_velocity[0];
    air_drag1[1] = -std::abs(cur_flap_hz) * air_drag_coefficient_[1] * body_velocity[1];
    air_drag1[2] = -std::abs(cur_flap_hz) * air_drag_coefficient_[2] * body_velocity[2];

    // joint_axis                                   = joint2_->GlobalAxis(0);
    // ignition::math::Vector3d body_velocity2_flap = body_velocity_2 - (body_velocity_2 * joint_axis) * joint_axis;

    // ignition::math::Vector3d air_drag2(0, 0, 0);
    // air_drag2[0] = -std::abs(cur_flap_hz) * air_drag_coefficient_[0] * body_velocity2[0];
    // air_drag2[1] = -std::abs(cur_flap_hz) * air_drag_coefficient_[1] * body_velocity2[1];
    // air_drag2[2] = -std::abs(cur_flap_hz) * air_drag_coefficient_[2] * body_velocity2[2];

    // if (counter++ <= 100) {
    link1_->AddLinkForce(ignition::math::Vector3d(0, 0, force / 2));
    link2_->AddLinkForce(ignition::math::Vector3d(0, 0, force / 2));
    // gzdbg << "the motor " << motor_number_ << " force is " << force << "\n";
    // } else {
    //     gzdbg << "the air_drag is : " << air_drag1[0] << ", " << air_drag1[1] << ", " << air_drag1[2] << "\n";
    // }

    // TODO:添加空气阻力

    link1_->AddLinkForce(air_drag1);
    link2_->AddLinkForce(air_drag1);

    turned_rad += turning_direction_ * real_motor_velocity * 0.002;
    // joint1_->SetPosition(0, turned_rad);
    // joint2_->SetPosition(0, -turned_rad);
}

void GazeboMotorModel::UpdateMotorFail()
{
    if (motor_number_ == motor_Failure_Number_ - 1) {
        // motor_constant_ = 0.0;
        joint1_->SetVelocity(0, 0);
        joint2_->SetVelocity(0, 0);
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

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}
