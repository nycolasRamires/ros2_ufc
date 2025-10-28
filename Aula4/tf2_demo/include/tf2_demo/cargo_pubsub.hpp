#ifndef CARGO_PUBSUB__CARGO_PUBSUB_HPP_
#define CARGO_PUBSUB__CARGO_PUBSUB_HPP_
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Dense>


namespace cargo_pubsub
{
    using namespace std::chrono_literals;
    class CargoTFNode : public rclcpp::Node{
        public:
            CargoTFNode()
            :
                Node("cargo_tf2_pub")
            {
                // ------!!! PARÂMETROS !!!-----
                this->declare_parameter<double>("cargo_distance", 0.5); // distância do cargo
                cargo_dist_ = this->get_parameter("cargo_distance").as_double();
                this->declare_parameter<std::string>("drone_tf_name", "drone_frame"); // distância do cargo
                drone_frame_ = this->get_parameter("drone_tf_name").as_string();

                //   Inicializa o broadcaster de transformações
                tf_broadcaster_ =
                    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                //   Inicializa o listener e buffer de transformação
                tf_buffer_ =
                    std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ =
                    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

                //timer
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&CargoTFNode::timer_cb, this)
                );
            }

        private:
            // Membros ROS
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //broadcaster de transformação
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // listener
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // buffer
            rclcpp::TimerBase::SharedPtr timer_;

            void timer_cb(); //função de callback do timer
            
            //parametros
            double cargo_dist_ = 0.0;
            std::string drone_frame_ = "drone_frame";

    };

}

#endif // CARGO_PUBSUB__CARGO_PUBSUB_HPP_