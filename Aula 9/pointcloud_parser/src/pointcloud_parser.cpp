#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>

using namespace std::placeholders;

class PointCloudParser : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  sensor_msgs::msg::PointCloud2 pc_;
  geometry_msgs::msg::TransformStamped tf2_msg_;
  double drone_height_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;

  tf2_ros::Buffer tfBuffer; // comp que guarda as transformações 
  tf2_ros::TransformListener tfListener; // TransformListener assina o tópico TF e enche tfBuffer
  

  void pointcloud_cb(const sensor_msgs::msg::PointCloud2 pc){
    pc_ = pc;
  }

  void parse_cloud(){
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_, "z");
    std::vector<int> temp_storage;
    
    // reserva de mem. para evitar realocacao (memory pool)
    temp_storage.reserve(pc_.width * pc_.height);

    for (size_t i = 0; i < (pc_.width * pc_.height); ++i, ++iter_z) {
        if (std::abs(*iter_z - drone_height_) < 0.5) {
            temp_storage.push_back((int)i);
        }
    }
    sensor_msgs::msg::PointCloud2 filtered_pc;
    filtered_pc.header = pc_.header; // Mantém o mesmo frame_id e timestamp
    filtered_pc.height = 1;
    filtered_pc.width = temp_storage.size();
    filtered_pc.fields = pc_.fields;
    filtered_pc.is_bigendian = pc_.is_bigendian;
    filtered_pc.point_step = pc_.point_step;
    filtered_pc.row_step = filtered_pc.width * filtered_pc.point_step;
    filtered_pc.is_dense = pc_.is_dense; 

    filtered_pc.data.resize(filtered_pc.row_step);

    const uint8_t* src_ptr = pc_.data.data(); 
    uint8_t* dst_ptr = filtered_pc.data.data();
    size_t p_size = pc_.point_step; // bytes em um ponto da nuvem

    for (size_t i = 0; i < temp_storage.size(); ++i) {
        int original_idx = temp_storage[i];

        std::memcpy(
            dst_ptr + (i * p_size),            
            src_ptr + (original_idx * p_size), 
            p_size                             
        );
    }
    pc_pub_->publish(filtered_pc);
  }

  void get_drone_z_to_map(){
    if (tfBuffer.canTransform("map", "drone_frame", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
      tf2_msg_ = tfBuffer.lookupTransform("map", "drone_frame", rclcpp::Time(0));
      drone_height_ = tf2_msg_.transform.translation.z;
      RCLCPP_INFO(this->get_logger(),"Altura do drone_frame (map): %.3f m", drone_height_);
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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1100),
                                        std::bind(&PointCloudParser::parse_cloud, this));

    timer2_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&PointCloudParser::get_drone_z_to_map, this));
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("nuvem_filtrada", 10);
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
