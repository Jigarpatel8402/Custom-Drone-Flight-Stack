#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <drone_interfaces/msg/drone_imu.hpp>
#include <drone_interfaces/msg/drone_gps.hpp>
#include <drone_interfaces/msg/drone_barometer.hpp>
#include <drone_interfaces/msg/drone_magnetometer.hpp>

#include <gz/transport/Node.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/magnetometer.pb.h>

class DroneSensorNode
{
public:

    DroneSensorNode()
    {
        ros_node_ =
            std::make_shared<rclcpp::Node>(
                "drone_sensor_node"
            );

        imu_pub_ =
            ros_node_->create_publisher<
                drone_interfaces::msg::DroneIMU
            >(
                "/drone_imu",
                10
            );

        gps_pub_ =
            ros_node_->create_publisher<
                drone_interfaces::msg::DroneGPS
            >(
                "/drone_gps",
                10
            );

        barometer_pub_ =
            ros_node_->create_publisher<
                drone_interfaces::msg::DroneBarometer
            >(
                "/drone_barometer",
                10
            );

        magnetometer_pub_ =
            ros_node_->create_publisher<
                drone_interfaces::msg::DroneMagnetometer
            >(
                "/drone_magnetometer",
                10
            );

        // gz_node_.Subscribe(
        //     "/world/iris_runway/model/iris/link/imu_link/sensor/imu_sensor/imu",
        //     &DroneSensorNode::imuCallback,
        //     this
        // );

        // Only for attitude testing
        gz_node_.Subscribe(
            "/world/iris_attitude_test_runway/model/iris_attitude_test/link/imu_link/sensor/imu_sensor/imu",
            &DroneSensorNode::imuCallback,
            this
        );

        gz_node_.Subscribe(
            "/navsat",
            &DroneSensorNode::gpsCallback,
            this
        );

        gz_node_.Subscribe(
            "/barometer",
            &DroneSensorNode::barometerCallback,
            this
        );

        gz_node_.Subscribe(
            "/magnetometer",
            &DroneSensorNode::magnetometerCallback,
            this
        );

        RCLCPP_INFO(
            ros_node_->get_logger(),
            "Drone Sensor Node Started"
        );
    }

    rclcpp::Node::SharedPtr node()
    {
        return ros_node_;
    }

private:

    void imuCallback(
        const gz::msgs::IMU & msg
    )
    {
        drone_interfaces::msg::DroneIMU imu_msg;

        imu_msg.accel_x =
            msg.linear_acceleration().x();

        imu_msg.accel_y =
            msg.linear_acceleration().y();

        imu_msg.accel_z =
            msg.linear_acceleration().z();

        imu_msg.gyro_x =
            msg.angular_velocity().x();

        imu_msg.gyro_y =
            msg.angular_velocity().y();

        imu_msg.gyro_z =
            msg.angular_velocity().z();

        imu_pub_->publish(
            imu_msg
        );
    }

    void gpsCallback(
        const gz::msgs::NavSat & msg
    )
    {
        drone_interfaces::msg::DroneGPS gps_msg;

        gps_msg.latitude =
            msg.latitude_deg();

        gps_msg.longitude =
            msg.longitude_deg();

        gps_msg.altitude =
            msg.altitude();

        gps_msg.velocity_north =
            msg.velocity_north();

        gps_msg.velocity_east =
            msg.velocity_east();

        gps_msg.velocity_up =
            msg.velocity_up();

        gps_pub_->publish(
            gps_msg
        );
    }

    void barometerCallback(
        const gz::msgs::FluidPressure & msg
    )
    {
        drone_interfaces::msg::DroneBarometer baro_msg;

        baro_msg.pressure =
            msg.pressure();

        barometer_pub_->publish(
            baro_msg
        );
    }

    void magnetometerCallback(
        const gz::msgs::Magnetometer & msg
    )
    {
        drone_interfaces::msg::DroneMagnetometer mag_msg;

        mag_msg.magnetic_x =
            msg.field_tesla().x();

        mag_msg.magnetic_y =
            msg.field_tesla().y();

        mag_msg.magnetic_z =
            msg.field_tesla().z();

        magnetometer_pub_->publish(
            mag_msg
        );
    }

private:

    gz::transport::Node gz_node_;

    rclcpp::Node::SharedPtr ros_node_;

    rclcpp::Publisher<
        drone_interfaces::msg::DroneIMU
    >::SharedPtr imu_pub_;

    rclcpp::Publisher<
        drone_interfaces::msg::DroneGPS
    >::SharedPtr gps_pub_;

    rclcpp::Publisher<
        drone_interfaces::msg::DroneBarometer
    >::SharedPtr barometer_pub_;

    rclcpp::Publisher<
        drone_interfaces::msg::DroneMagnetometer
    >::SharedPtr magnetometer_pub_;
};

int main(
    int argc,
    char ** argv
)
{
    rclcpp::init(
        argc,
        argv
    );

    auto sensor_node =
        std::make_shared<
            DroneSensorNode
        >();

    rclcpp::spin(
        sensor_node->node()
    );

    rclcpp::shutdown();

    return 0;
}
