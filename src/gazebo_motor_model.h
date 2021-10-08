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

#include <stdio.h>

#include "EscSensor.pb.h"
#include "Float.pb.h"
#include "MotorCommand.pb.h"
#include "MotorSpeed.pb.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rotors_model/motor_model.hpp>

#include "common.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW  = -1;
}
namespace rotor_velocity_units {
const static std::string RPM            = "rpm";
const static std::string RAD_PER_SECOND = "rad/s";
}

namespace gazebo {
// Default values
static const std::string kDefaultNamespace               = "";
static const std::string kDefaultCommandSubTopic         = "/aircraft/command/motor";
static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
static const std::string kDefaultMotorVelocityPubTopic   = "/aircraft/sensor/esc";

typedef const boost::shared_ptr<const cmd_msgs::msgs::MotorCommand> MotorCommandPtr;

/*
// Protobuf test
typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
static const std::string kDefaultMotorTestSubTopic = "motors";
*/

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaulteFlapMaxRad              = 1.05;
static constexpr double kDefaultMotorCr                  = 12.2;
static constexpr double kDefaultMotorWb                  = -1.195;
static constexpr double kDefaultMotorT                   = 0.01613;
static constexpr double kDefaultMotorJm                  = 0.0001287;
static constexpr double kDefaultInitRPM                  = 557.142;
static constexpr double kDefaultRotorMaxFlapHz           = 60;
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

class GazeboMotorModel : public MotorModel, public ModelPlugin {
public:
    GazeboMotorModel()
        : ModelPlugin()
        , MotorModel()
        , command_sub_topic_(kDefaultCommandSubTopic)
        , motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic)
        , motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic)
        , motor_number_(0)
        , motor_Failure_Number_(0)
        , turning_direction_(turning_direction::CW)
        , flap_max_rad_(kDefaulteFlapMaxRad)
        , motor_cr_(kDefaultMotorCr)
        , motor_wb_(kDefaultMotrorWb)
        , motor_T_(kDefaultMotorT)
        , motor_jm_(kDefaultMotorJm)
        , init_RPM_(kDefaultInitRPM)
        , max_flap_hz(kDefaultRotorMaxFlapHz)
        , ref_flap_hz(0.0)
        , cur_flap_hz(0.0)
        , rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim)
        , rotor_velocity_units_(rotor_velocity_units::RAD_PER_SECOND)
        , turned_rad(0.0)
    {
    }

    virtual ~GazeboMotorModel();

    virtual void InitializeParams();
    virtual void Publish();
    // void testProto(MotorSpeedPtr &msg);
protected:
    virtual void UpdateForcesAndMoments();
    /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
    /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
    virtual void   UpdateMotorFail();
    virtual void   Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void   OnUpdate(const common::UpdateInfo& /*_info*/);
    virtual void   Reset();
    virtual double CommandTransferFunction(double x);

private:
    std::string command_sub_topic_;
    std::string motor_failure_sub_topic_;
    std::string joint1_name_;
    std::string joint2_name_;
    std::string link1_name_;
    std::string link2_name_;
    std::string motor_speed_pub_topic_;
    std::string namespace_;
    std::string rotor_velocity_units_;

    int motor_number_;
    int turning_direction_;

    int motor_Failure_Number_; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish
                                  accordingly */
    int tmp_motor_num;         // A temporary variable used to print msg

    int screen_msg_flag = 1;

    double flap_max_rad_;
    double motor_cr_;
    double motor_wb_;
    double motor_T_;
    double motor_jm_;
    double init_RPM_;
    double max_flap_hz;
    double rotor_velocity_slowdown_sim_;
    double ref_flap_hz;
    double cur_flap_hz;
    double motor_tor_vel;
    double current_force_  = 0;
    double current_torque_ = 0;
    double motor_dynamic_flap_hz_[2]; // Max 3 terms
    double flap_hz_to_thrust_[3];
    double air_drag_coefficient_[2];
    double cop_poistion_[3];

    double turned_rad;

    sensor_msgs::msgs::EscSensor sensor;

    transport::NodePtr       node_handle_;
    transport::PublisherPtr  esc_sensor_pub_;
    transport::SubscriberPtr command_sub_;
    transport::SubscriberPtr motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to
                                                    fail, as an integer */

    physics::ModelPtr model_;
    physics::JointPtr joint1_;
    physics::JointPtr joint2_;
    physics::LinkPtr  link1_;
    physics::LinkPtr  link2_;

    bool        use_pid_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection_;

    boost::thread         callback_queue_thread_;
    void                  QueueThread();
    std_msgs::msgs::Float turning_velocity_msg_;
    void                  VelocityCallback(MotorCommandPtr& _cmd);
    void                  MotorFailureCallback(
                         const boost::shared_ptr<const msgs::Int>& fail_msg); /*!< Callback for the motor_failure_sub_ subscriber */
    std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
    /*
      // Protobuf test
      std::string motor_test_sub_topic_;
      transport::SubscriberPtr motor_sub_;
    */
};
}
