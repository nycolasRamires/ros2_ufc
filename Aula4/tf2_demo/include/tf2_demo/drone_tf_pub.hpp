#ifndef DRONE_TF_PUB__DRONE_TF_PUB_HPP_
#define DRONE_TF_PUB__DRONE_TF_PUB_HPP_
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Dense>


namespace drone_tf_pub
{
    using namespace std::chrono_literals;
    class DroneTFPublisher : public rclcpp::Node{
        public:
            DroneTFPublisher() : Node("drone_pub")
            {
                this->declare_parameter<double>("speed", 1.0); //velocidade do frame
                this->declare_parameter<std::string>("drone_tf_name","drone_frame"); // nome do frame do drone
                drone_frame_ = this->get_parameter("drone_tf_name").as_string();

                //   Inicializa o broadcaster de transformação
                tf_broadcaster_ =
                    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
                //construtor do tf2_ros::TransformBroadcaster precisa de uma ref de um rclcpp::Node

                //timer
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&DroneTFPublisher::timer_cb, this)
                );
                last_time_ = this->get_clock()->now().seconds();
            }

        private:
            // Membros ROS
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //broadcaster da transformação usa um shared_ptr
            rclcpp::TimerBase::SharedPtr timer_;

            void timer_cb(); //função de callback do timer
            void viviani_calculator_(const double theta); // calculador dos parametros linear, angular do frame

            double theta_; // ângulo da curva paramétrica
            double last_time_=0.0;
            
            // Parâmetros
            double speed_; // velocidade do frame ao longo da curva
            std::string drone_frame_="drone_frame";

    };

}

#endif // DRONE_TF_PUB__DRONE_TF_PUB_HPP_