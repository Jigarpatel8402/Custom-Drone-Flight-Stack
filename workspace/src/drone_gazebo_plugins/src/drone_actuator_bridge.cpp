#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/Link.hh>
#include "drone_gazebo_plugins/rotor_state.hpp"
#include "drone_gazebo_plugins/rotor_physics.hpp"
#include "drone_interfaces/msg/drone_rotor.hpp"
#include <iostream>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

namespace drone_gazebo_bridge
{
    class DroneActuatorBridge :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
        private:
            gz::sim::Entity base_link;
            gz::sim::Entity model_entity;

            gz::sim::Entity rotor_joint[4];
            gz::sim::Entity rotor_link[4];

            rclcpp::Node::SharedPtr ros_node;

            rclcpp::Subscription<
                drone_interfaces::msg::DroneRotor
            >::SharedPtr pwm_sub;

            RotorState rotors[4];
            RotorPhysics rotorPhysics;

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
                        "drone_actuator_bridge"
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
                        this->rotor_joint[0] = _ent;

                    else if(_name->Data() == "rotor_1_joint")
                        this->rotor_joint[1] = _ent;

                    else if(_name->Data() == "rotor_2_joint")
                        this->rotor_joint[2] = _ent;
                    
                    else if(_name->Data() == "rotor_3_joint")
                        this->rotor_joint[3] = _ent;
                    
                    else if(_name->Data() == "rotor_0")
                        this->rotor_link[0] = _ent;
                    
                    else if(_name->Data() == "rotor_1")
                        this->rotor_link[1] = _ent;
                    
                    else if(_name->Data() == "rotor_2")
                        this->rotor_link[2] = _ent;
                    
                    else if(_name->Data() == "rotor_3")
                        this->rotor_link[3] = _ent;

                    return true;
                });

                this->rotors[0] = RotorState(0, +1);
                this->rotors[1] = RotorState(1, -1);
                this->rotors[2] = RotorState(2, +1);
                this->rotors[3] = RotorState(3, -1);
                
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
                        this->rotors[0].pwm = msg->rotor0;
                        this->rotors[1].pwm = msg->rotor1;
                        this->rotors[2].pwm = msg->rotor2;
                        this->rotors[3].pwm = msg->rotor3;
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

                double dt = std::chrono::duration<double>(
                    _info.dt
                ).count();

                UpdateMotors(dt);

                UpdateJointVelocity(_ecm);

                ApplyRotorWrenches(_ecm);

                static int counter = 0;
                counter++;
                
                if (counter % 100 == 0)
                {
                    std::cout
                        << "PWM: "
                        << rotors[0].pwm << " "
                        << rotors[1].pwm << " "
                        << rotors[2].pwm << " "
                        << rotors[3].pwm
                        << std::endl;
                    
                    std::cout
                        << "Throttle: "
                        << rotors[0].throttle << " "
                        << rotors[1].throttle << " "
                        << rotors[2].throttle << " "
                        << rotors[3].throttle
                        << std::endl;
                    
                    std::cout
                        << "Omega: "
                        << rotors[0].currentOmega << " "
                        << rotors[1].currentOmega << " "
                        << rotors[2].currentOmega << " "
                        << rotors[3].currentOmega
                        << std::endl;
                    
                    std::cout
                        << "Reactive Torque: "
                        << rotors[0].reactiveTorque << " "
                        << rotors[1].reactiveTorque << " "
                        << rotors[2].reactiveTorque << " "
                        << rotors[3].reactiveTorque
                        << std::endl;
                }
            }
            
            void UpdateMotors(double dt)
            {
                for (auto &rotor: rotors)
                {
                    rotorPhysics.Update(rotor, dt);
                }
            }

            void UpdateJointVelocity(
                gz::sim::EntityComponentManager &_ecm
            )
            {
                for (int i = 0; i < 4; i++)
                {
                    double speed = -rotors[i].direction * rotors[i].currentOmega;

                    auto cmd = 
                        _ecm.Component<
                            gz::sim::components::JointVelocityCmd
                        >(rotor_joint[i]);
                    
                    if(!cmd)
                    {
                        _ecm.CreateComponent(
                            this->rotor_joint[i],
                            gz::sim::components::JointVelocityCmd(
                                {speed}
                            )
                        );
                    }
                    else
                    {
                        cmd->Data()[0] = speed;
                    }
                }
            }
            
            void ApplyRotorWrenches(
                gz::sim::EntityComponentManager &_ecm
            )
            {
                for (int i = 0; i < 4; i ++)
                {
                    gz::sim::Link link(rotor_link[i]);

                    if (!link.Valid(_ecm))
                        continue;
                    
                    std::optional<gz::math::Pose3d> pose = link.WorldPose(_ecm);

                    if (!pose)
                        continue;
                    
                    gz::math::Vector3d localThrust(0.0, 0.0, rotors[i].thrust);
                    gz::math::Vector3d localTorque(0.0, 0.0, rotors[i].reactiveTorque);

                    gz::math::Quaterniond q = pose->Rot();

                    gz::math::Vector3d worldThrust = q.RotateVector(localThrust);
                    gz::math::Vector3d worldTorque = q.RotateVector(localTorque);

                    link.AddWorldWrench(
                        _ecm,
                        worldThrust,
                        worldTorque
                    );
                }
            }
    };
}

GZ_ADD_PLUGIN(
    drone_gazebo_bridge::DroneActuatorBridge,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)
