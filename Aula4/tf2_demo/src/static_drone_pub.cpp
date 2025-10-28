#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cargo_static_pub{

  class CargoFramePublisher : public rclcpp::Node
  {
  public:
    explicit CargoFramePublisher(char * transformation[], std::vector<double> xyz_vals = {0.0, 0.0, 0.0} )
    : Node("static_cargo_tf2_broadcaster")
    {
      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Publish static transforms once at startup
      this->make_transform(transformation, xyz_vals);
    }

  private:
    void make_transform(char * transformation[], std::vector<double> xyz_vals)
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = transformation[1]; // nome do frame do 'drone'
      t.child_frame_id = transformation[2]; // nome do frame da carga

      t.transform.translation.x = xyz_vals[0];
      t.transform.translation.y = xyz_vals[1];
      t.transform.translation.z = xyz_vals[2];
      tf2::Quaternion q;
      q.setRPY(0.0,0.0,0.0);
      t.transform.rotation = tf2::toMsg(q);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_static_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  };
}

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Obtém os parâmetros da linha de comando
  if (argc < 3 && argc>6) {
    RCLCPP_INFO(
      logger,
      "Uso: ros2 run tf2_demo cargo_publisher drone_frame_name cargo_frame_name [x] [y] [z]\n"
      "Argumentos x, y, z são opcionais (padrão = 0.0).");
    return 1;
  }

  // Valor padrão dos arguumentos x y z
  std::vector<double> xyz_args = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //x,y,z

  // obtém argumentos opcionais
  for (int i = 0; i < 1 && (i + 3) < argc; ++i) {
    xyz_args[i] = std::atof(argv[i + 3]);
  }

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cargo_static_pub::CargoFramePublisher>(argv, xyz_args));
  rclcpp::shutdown();
  return 0;
}