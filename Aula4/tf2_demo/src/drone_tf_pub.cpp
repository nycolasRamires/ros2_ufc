#include "tf2_demo/drone_tf_pub.hpp"

namespace drone_tf_pub
{
    void DroneTFPublisher::timer_cb(){
    // Pega tempo decorrido entre o último callback e atual
    auto current_time_ = this->get_clock()->now().seconds();
    double dt = current_time_ - last_time_;
    last_time_ = current_time_;

    // Atualiza o parâmetro de comprimento de arco do frame
    theta_ = this->get_parameter("speed").as_double()*dt + theta_;
    viviani_calculator_(theta_);
  }

  void DroneTFPublisher::viviani_calculator_(const double th){
    // Calcula a curva de viviani e retorna uma mensagem de transformação

    // Posicao
    double st = sin(th); double ct = cos(th); // sin e cos são operações caras - calculamos uma vez só
    double s2t = sin(2*th); double c2t = cos(2*th);
    double x = st*ct;
    double y = st*st;
    double z = ct;

    // Eixos do frame de frenet-serret
    // double kappa = sqrt(3*pow(st,2) + 5)/pow((pow(st,2) + 1),3/2);
    Eigen::Vector3d dr(
       c2t,
       s2t,
       -st
    );
    Eigen::Vector3d T = dr.normalized(); // Tangente normalizada: divide por |dr|
    Eigen::Vector3d d2r(
      -2*s2t,
      2*c2t,
      -ct
    );

    // Binormal B
    auto drxd2r = dr.cross(d2r);
    Eigen::Vector3d B = drxd2r.normalized(); //Binormal

    // Normal N
    Eigen::Vector3d N = B.cross(T).normalized();

    // Monta o frame de Frenet-Serret
    tf2::Matrix3x3 fs_frame(
        T.x(), N.x(), B.x(),
        T.y(), N.y(), B.y(),
        T.z(), N.z(), B.z()
    );
    tf2::Quaternion q;
    fs_frame.getRotation(q); //transforma matriz de rotação em quaternion
    q.normalize();

    // Cria e preenche a mensagem de transformação
    geometry_msgs::msg::TransformStamped viviani_tf;
    viviani_tf.header.stamp = this->get_clock()->now();
    viviani_tf.header.frame_id = "center";
    viviani_tf.child_frame_id = drone_frame_;

    viviani_tf.transform.translation.x = x;
    viviani_tf.transform.translation.y = y;
    viviani_tf.transform.translation.z = z;

    viviani_tf.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(viviani_tf);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<drone_tf_pub::DroneTFPublisher>());
  rclcpp::shutdown();
  return 0;
}
