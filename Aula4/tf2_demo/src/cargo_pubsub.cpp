#include "tf2_demo/cargo_pubsub.hpp"

namespace cargo_pubsub
{
    void CargoTFNode::timer_cb(){
    
    // Nomes dos frames que usaremos para buscar a transformada
    std::string fromFrameRel = "map";
    std::string toFrameRel = drone_frame_;


    //Busca pelas TFs desejadas no cache
    geometry_msgs::msg::TransformStamped tf_map2drone;
    try {
        // Busca transformada entre frames toFrameRel e fromFrameRel
        tf_map2drone = tf_buffer_->lookupTransform(
        fromFrameRel, toFrameRel,
        tf2::TimePointZero);
        // tf2::TimePointZero pega a mais recente disponível

    } catch (const tf2::TransformException & ex) {
        // se não estiver disponível, não publica nada - retorna o callback
        auto& clk = *this->get_clock();
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), clk, 1000,
        "Não foi enccontrada TF de %s para %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what()
        ); // Throttle de 1000 ms - evita spammar o logger
        return;
    }

    ///// Lógica por trás da matemátia a seguir:
    // Queremos publicar o frame cargo (C), que esteja a -0.5 metros de distancia do drone (D), no sentido z do frame map (M), e com orientação igual à de M
    // Portanto, queremos que a posição desse frame no sistema M seja:
    //  p_M2C = [p_M2C_x; p_M2C_y; p_M2C_z] = [p_M2D_x; p_M2D_y; p_M2D_z - 0.5] = p_M2D - [0; 0; 0.5]
    // Usamos isso como origem do frame e uma rotação identidade para construir TF_M2C
    // Sabendo então a TF de D com relação a M (TF_M2D), podemos inverter para obter TF_D2C:
    //    TF_D2C = (TF_M2D).inv() * TF_M2C
    
    tf2::Transform TF_M2D;
    tf2::fromMsg(tf_map2drone.transform,TF_M2D);
    tf2::Vector3 p_M2C = TF_M2D.getOrigin() + tf2::Vector3(0.0, 0.0, -0.5); // Posição do drone_frame com relação a map
    tf2::Quaternion q_M2C(0, 0, 0, 1); // Orientação do cargo é a mesma do map (identidade)
    tf2::Transform TF_M2C(q_M2C,p_M2C); // Constroi frame de Mapa -> Cargo
    tf2::Transform TF_D2C = TF_M2D.inverse() * TF_M2C;


    // Cria e preenche a mensagem de transformação
    geometry_msgs::msg::TransformStamped cargo_tf;
    // cargo_tf.header.stamp = this->get_clock()->now();
    cargo_tf.header.stamp = tf_map2drone.header.stamp;
    cargo_tf.header.frame_id = toFrameRel;
    cargo_tf.child_frame_id = "cargo_dynamic_frame";

    // Preenche a transformada com os campos que calculamos
    cargo_tf.transform = tf2::toMsg(TF_D2C);

    tf_broadcaster_->sendTransform(cargo_tf);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<cargo_pubsub::CargoTFNode>());
  rclcpp::shutdown();
  return 0;
}
