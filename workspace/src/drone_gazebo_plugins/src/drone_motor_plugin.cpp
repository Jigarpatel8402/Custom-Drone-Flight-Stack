#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include "drone_interfaces/msg/drone_rotor.hpp"
#include <iostream>
#include <algorithm>

namespace drone_gazebo_plugins
{
    class DroneMotorPlugin :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
        private:
            gz::sim::Entity base_link;
            gz::sim::Entity model_entity;

            gz::sim::Entity rotor0_joint = 42;
            gz::sim::Entity rotor1_joint = 43;
            gz::sim::Entity rotor2_joint = 44;
            gz::sim::Entity rotor3_joint = 45;

            rclcpp::Node::SharedPtr ros_node;

            rclcpp::Subscription<
                drone_interfaces::msg::DroneRotor
            >::SharedPtr pwm_sub;

            float pwm0 = 1000.0f;
            float pwm1 = 1000.0f;
            float pwm2 = 1000.0f;
            float pwm3 = 1000.0f;

            float motor0_rpm = 0.0f;
            float motor1_rpm = 0.0f;
            float motor2_rpm = 0.0f;
            float motor3_rpm = 0.0f;

            float max_rpm = 12000.0f;

            float target_motor0_rpm = 0.0f;
            float target_motor1_rpm = 0.0f;
            float target_motor2_rpm = 0.0f;
            float target_motor3_rpm = 0.0f;

            gz::sim::Entity rotor0_link;
            gz::sim::Entity rotor1_link;
            gz::sim::Entity rotor2_link;
            gz::sim::Entity rotor3_link;

            // float total_thrust = 15;       // for hover total_thrust = 17.1548
        
        public:

            void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr
            ) override
            {
                this->model_entity = _entity;
                gz::sim::Model model(_entity);

                if(!rclcpp::ok())
                {
                    int argc = 0;
                    char **argv = nullptr;

                    rclcpp::init(argc, argv);
                }

                this->ros_node =
                    rclcpp::Node::make_shared(
                        "drone_motor_plugin"
                    );
                
                std::cout
                    << "ROS Node Created!"
                    << std::endl;

                std::cout
                    << "Model Valid = "
                    << model.Valid(_ecm)
                    << std::endl;

                _ecm.Each<gz::sim::components::Name>(
                    [&](const gz::sim::Entity &_ent,
                        const gz::sim::components::Name *_name)
                {
                    if(_name->Data() == "base_link")
                        this->base_link = _ent;

                    else if(_name->Data() == "rotor_0_joint")
                        this->rotor0_joint = _ent;

                    else if(_name->Data() == "rotor_1_joint")
                        this->rotor1_joint = _ent;

                    else if(_name->Data() == "rotor_2_joint")
                        this->rotor2_joint = _ent;

                    else if(_name->Data() == "rotor_3_joint")
                        this->rotor3_joint = _ent;
                    
                    else if(_name->Data() == "rotor_0")
                        this->rotor0_link = _ent;

                    else if(_name->Data() == "rotor_1")
                        this->rotor1_link = _ent;

                    else if(_name->Data() == "rotor_2")
                        this->rotor2_link = _ent;

                    else if(_name->Data() == "rotor_3")
                        this->rotor3_link = _ent;

                    return true;
                });
                
                this->pwm_sub =
                this->ros_node->create_subscription<
                    drone_interfaces::msg::DroneRotor
                >(
                    "/pwm_output",
                    10,
                    [this](
                        const drone_interfaces::msg::DroneRotor::SharedPtr msg
                    )
                    {
                        this->pwm0 = msg->rotor0;
                        this->pwm1 = msg->rotor1;
                        this->pwm2 = msg->rotor2;
                        this->pwm3 = msg->rotor3;

                        if (this->pwm0 < 1000.0f)
                        {
                            this->pwm0 = 1000.0f;
                        }
                        if (this->pwm0 > 2000.0f)
                        {
                            this->pwm0 = 2000.0f;
                        }
                        if (this->pwm1 < 1000.0f)
                        {
                            this->pwm1 = 1000.0f;
                        }
                        if (this->pwm1 > 2000.0f)
                        {
                            this->pwm1 = 2000.0f;
                        }
                        if (this->pwm2 < 1000.0f)
                        {
                            this->pwm2 = 1000.0f;
                        }
                        if (this->pwm2 > 2000.0f)
                        {
                            this->pwm2 = 2000.0f;
                        }
                        if (this->pwm3 < 1000.0f)
                        {
                            this->pwm3 = 1000.0f;
                        }
                        if (this->pwm3 > 2000.0f)
                        {
                            this->pwm3 = 2000.0f;
                        }

                        float throttle0 =
                            (this->pwm0 - 1000.0f) / 1000.0f;

                        float throttle1 =
                            (this->pwm1 - 1000.0f) / 1000.0f;

                        float throttle2 =
                            (this->pwm2 - 1000.0f) / 1000.0f;

                        float throttle3 =
                            (this->pwm3 - 1000.0f) / 1000.0f;

                        throttle0 = std::clamp(throttle0, 0.0f, 1.0f);
                        throttle1 = std::clamp(throttle1, 0.0f, 1.0f);
                        throttle2 = std::clamp(throttle2, 0.0f, 1.0f);
                        throttle3 = std::clamp(throttle3, 0.0f, 1.0f);

                        this->target_motor0_rpm =
                            throttle0 * this->max_rpm;

                        this->target_motor1_rpm =
                            throttle1 * this->max_rpm;

                        this->target_motor2_rpm =
                            throttle2 * this->max_rpm;

                        this->target_motor3_rpm =
                            throttle3 * this->max_rpm;
                    }
                );
            }

            void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm
            ) override
            {
                if(this->ros_node)
                {
                    rclcpp::spin_some(
                        this->ros_node
                    );
                }

                float alpha = 0.02f;

                this->motor0_rpm += alpha * (this->target_motor0_rpm - this->motor0_rpm);
                this->motor1_rpm += alpha * (this->target_motor1_rpm - this->motor1_rpm);
                this->motor2_rpm += alpha * (this->target_motor2_rpm - this->motor2_rpm);
                this->motor3_rpm += alpha * (this->target_motor3_rpm - this->motor3_rpm);

                double rotor0_speed =
                    this->motor0_rpm * 2.0 * M_PI / 60.0;

                double rotor1_speed =
                    (-this->motor1_rpm) * 2.0 * M_PI / 60.0;

                double rotor2_speed =
                    this->motor2_rpm * 2.0 * M_PI / 60.0;
                
                double rotor3_speed =
                    (-this->motor3_rpm) * 2.0 * M_PI / 60.0;

                auto cmd0 =
                    _ecm.Component<
                        gz::sim::components::JointVelocityCmd
                    >(this->rotor0_joint);
                
                auto cmd1 =
                    _ecm.Component<
                        gz::sim::components::JointVelocityCmd
                    >(this->rotor1_joint);

                auto cmd2 =
                    _ecm.Component<
                        gz::sim::components::JointVelocityCmd
                    >(this->rotor2_joint);

                auto cmd3 =
                    _ecm.Component<
                        gz::sim::components::JointVelocityCmd
                    >(this->rotor3_joint);

                if(!cmd0)
                {
                    _ecm.CreateComponent(
                        this->rotor0_joint,
                        gz::sim::components::JointVelocityCmd(
                            {rotor0_speed}
                        )
                    );
                }
                else
                {
                    cmd0->Data()[0] =
                        rotor0_speed;
                }

                if(!cmd1)
                {
                    _ecm.CreateComponent(
                        this->rotor1_joint,
                        gz::sim::components::JointVelocityCmd(
                            {rotor1_speed}
                        )
                    );
                }
                else
                {
                    cmd1->Data()[0] =
                        rotor1_speed;
                }

                if(!cmd2)
                {
                    _ecm.CreateComponent(
                        this->rotor2_joint,
                        gz::sim::components::JointVelocityCmd(
                            {rotor2_speed}
                        )
                    );
                }
                else
                {
                    cmd2->Data()[0] =
                        rotor2_speed;
                }

                if(!cmd3)
                {
                    _ecm.CreateComponent(
                        this->rotor3_joint,
                        gz::sim::components::JointVelocityCmd(
                            {rotor3_speed}
                        )
                    );
                }
                else
                {
                    cmd3->Data()[0] =
                        rotor3_speed;
                }

                static int counter = 0;
                static int i = 30;
                counter++;
                
                if (this->motor0_rpm != 0 && counter % 100 == 0)
                {
                    i--;
                    std::cout
                        << "Rotor RPM = "
                        << this->motor0_rpm
                        << " "
                        << this->motor1_rpm
                        << " "
                        << this->motor2_rpm
                        << " "
                        << this->motor3_rpm
                        << std::endl;
                }

                // if(this->motor0_rpm != 0 && counter % 400 == 0)
                // {
                //     std::cout
                //         << "Rotor RPM = "
                //         << this->motor0_rpm
                //         << " "
                //         << this->motor1_rpm
                //         << " "
                //         << this->motor2_rpm
                //         << " "
                //         << this->motor3_rpm
                //         << std::endl;
                    
                //     std::cout
                //         << "Cmd0 = "
                //         << rotor0_speed
                //         << std::endl;

                //     std::cout
                //         << "Cmd1 = "
                //         << rotor1_speed
                //         << std::endl;

                //     std::cout
                //         << "Cmd2 = "
                //         << rotor2_speed
                //         << std::endl;

                //     std::cout
                //         << "Cmd3 = "
                //         << rotor3_speed
                //         << std::endl;

                //     auto pose0 =
                //         _ecm.Component<
                //             gz::sim::components::WorldPose
                //         >(this->rotor0_link);

                //     if (pose0)
                //     {
                //         auto rot = pose0->Data().Rot();

                //         std::cout
                //             << "Rotor0 Roll Pitch Yaw = "
                //             << gz::math::Angle(rot.Roll()).Degree()
                //             << " "
                //             << gz::math::Angle(rot.Pitch()).Degree()
                //             << " "
                //             << gz::math::Angle(rot.Yaw()).Degree()
                //             << std::endl;
                //     }
                // }
            }
    };
}

GZ_ADD_PLUGIN(
    drone_gazebo_plugins::DroneMotorPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)
