#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::placeholders;

class PointCloudParser : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
  sensor_msgs::msg::PointCloud2 pc_;
  geometry_msgs::msg::TransformStamped tf2_msg;
  double drone_height;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;

  tf2_ros::Buffer tfBuffer; // comp que guarda as transformações 
  tf2_ros::TransformListener tfListener; // TransformListener assina o tópico TF e enche tfBuffer
  

  void pointcloud_cb(const sensor_msgs::msg::PointCloud2 pc){
    pc_ = pc;
  }

  void get_second_point(){
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_, "z");

    iter_x += 1; //o iterator (ponteiro) começa no primeiro ponto, estou pegando o segundo
    iter_y += 1;
    iter_z += 1;

    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    RCLCPP_INFO(this->get_logger(),"segundo ponto : %f %f %f", x,y,z);

  }

  void get_drone_z_to_map(){
    if (tfBuffer.canTransform("map", "drone_frame", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
      tf2_msg = tfBuffer.lookupTransform("map", "drone_frame", rclcpp::Time(0));
      drone_height = tf2_msg.transform.translation.z;
      RCLCPP_INFO(this->get_logger(),"Altura do drone_frame (map): %.3f m", drone_height);
    }
  }

public:
  PointCloudParser(): Node("pointcloud_parser"),
    tfBuffer(this->get_clock()),     // init do buffer 
    tfListener(tfBuffer) // init to listener das tfs
    {
    pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("ex_point_cloud",10,
                                                    std::bind(&PointCloudParser::pointcloud_cb, 
                                                    this, _1));
    timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                        std::bind(&PointCloudParser::get_second_point, this));

    timer2_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&PointCloudParser::get_drone_z_to_map, this));
    }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudParser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
