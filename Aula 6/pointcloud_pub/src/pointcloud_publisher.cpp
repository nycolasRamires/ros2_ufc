#include <chrono>
#include <memory>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("pointcloud_publisher"),
    dist_(0.0, 1.0),
    mt_(std::random_device{}())
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ex_point_cloud", 10);

    // timer de 1 s
    timer_ = this->create_wall_timer(
      1s, std::bind(&PointCloudPublisher::on_timer, this));

    last_update_time_ = this->now();
    generate_points();  // inicial
  }

private:
  void generate_points()
  {
    points_.clear();
    const double center_x = 3.0, center_y = -1.0, center_z = 2.0;
    const double radius = 2.0;
    const int n = 500;

    std::uniform_real_distribution<double> d(-radius, radius);

    for (int i = 0; i < n; ++i) {
      double x, y, z;
      do {
        x = d(mt_);
        y = d(mt_);
        z = d(mt_);
      } while (x*x + y*y + z*z > radius*radius);
      points_.push_back({ static_cast<float>(center_x + x),
                          static_cast<float>(center_y + y),
                          static_cast<float>(center_z + z) });
    }
  }

  sensor_msgs::msg::PointCloud2 make_pointcloud2()
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.height = 1;
    msg.width  = static_cast<uint32_t>(points_.size());
    msg.is_bigendian = false;
    msg.is_dense = false;

    // define campos x, y, z
    sensor_msgs::msg::PointField field_x;
    field_x.name     = "x";
    field_x.offset   = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count    = 1;

    sensor_msgs::msg::PointField field_y = field_x;
    field_y.name   = "y";
    field_y.offset = 4;

    sensor_msgs::msg::PointField field_z = field_x;
    field_z.name   = "z";
    field_z.offset = 8;

    msg.fields = { field_x, field_y, field_z };
    msg.point_step = 12; // 3 points of 4 bytes 
    msg.row_step   = msg.point_step * msg.width;

    // allocate data
    msg.data.resize(msg.point_step * msg.width);
    // fill data
    for (size_t i = 0; i < points_.size(); ++i) {
      float px = points_[i][0];
      float py = points_[i][1];
      float pz = points_[i][2];
      memcpy(&msg.data[i * msg.point_step + 0], &px, sizeof(float));
      memcpy(&msg.data[i * msg.point_step + 4], &py, sizeof(float));
      memcpy(&msg.data[i * msg.point_step + 8], &pz, sizeof(float));
    }

    return msg;
  }

  void on_timer()
  {
    auto now = this->now();
    if ((now - last_update_time_).seconds() >= 2.0) {
      generate_points();
      last_update_time_ = now;
    }
    auto msg = make_pointcloud2();
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published %zu points", points_.size());
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::array<float,3>> points_;
  rclcpp::Time last_update_time_;

  std::uniform_real_distribution<double> dist_;
  std::mt19937 mt_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
