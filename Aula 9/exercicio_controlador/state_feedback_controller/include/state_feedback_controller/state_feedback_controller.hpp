/// Definição de um controlador por feedback de estados para o exemplo de controle de um carrinho deslizante passado em aula.
// Documentação da classe base de ControllerInterface: 
//      https://control.ros.org/jazzy/doc/api/classcontroller__interface_1_1ControllerInterfaceBase.html
// Código modificado a partir do exemplo 7 do pacote de demonstração do ros2_control.
//
// Programador: Gustavo Comerlato

#ifndef STATE_FEEDBACK_CONTROLLER__STATE_FEEDBACK_CONTROLLER_HPP_
#define STATE_FEEDBACK_CONTROLLER__STATE_FEEDBACK_CONTROLLER_HPP_

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include <controller_interface/controller_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <realtime_tools/realtime_buffer.hpp>

#include <state_feedback_controller/visibility_control.hpp>

#include "eigen3/Eigen/Core"

namespace state_feedback_controller
{
class StateFeedbackController : public controller_interface::ControllerInterface
{
  // A classe ControllerInterfaceBase tem as variáveis membros state_interfaces_ e command_interfaces_,
  //  que são std::vectors que disponibilizam acesso aos dados das HardwareInterfaces.
  // A ordem de interfaces nesses vetores é determinada pelo valor retornado pela nossa implementação
  //  dos métodos {state,command}_interface_configuration()


  ////// Parâmetros
  Eigen::MatrixXd K_; // Matriz de ganhos de feedback de estados
  std::vector<std::string> jointNames_; 
  int priority_; 

  // Variáveis membros
  Eigen::VectorXd q_;  // Posições atuais das juntas
  Eigen::VectorXd dq_; // Velocidades atuais das juntas
  Eigen::VectorXd qr_;  // Posições de referência das juntas
  Eigen::VectorXd tau_; // Esforços aplicados nas juntas


  // Subscriber e estruturas para interface via tópicos.

  // O sistema ros2_control vem com um conjunto de ferramentas de tempo real (realtime_tools),
  //  para facilitar a troca de mensagens entre diferentes threads.
  // Aqui usamos o RealtimeBuffer para receber dados de uma thread não real-time com a thread do controlador.
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr sub_command_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>> referencePoint_;

  // Função de callback usual para subscriber
  void commandCB(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr referencePoint);

  // Interfaces que o controlador requisitará e usará durante o laço de atualização.
  // Aqui temos um vetor (uma interface por junta), apesar de que neste caso não seria necessário.
  // Deixamos dessa forma por generalidade (ex., caso alguém queira extendê-lo para um robô multi-juntas). 
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>  joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>    joint_velocity_state_interface_;

  // Mapas (para podermos configurar os nomes das interfaces conforme desejarmos)
  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"effort", &joint_effort_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_}};

  // Outras funções e variáveis auxiliares
  double get_hw_value(const hardware_interface::LoanedStateInterface & interface, const unsigned int max_tries=10);

  public:
  STATE_FEEDBACK_CONTROLLER_PUBLIC
  StateFeedbackController(void); // Construtor default de classe. O processo de construção também é realizado no método on_init().

  // Abaixo: métodos que são sobrescritos da classe base ControllerInterface.
  //  Segue também uma breve descrição do que cada método deve fazer.


  // Método on_init: chamado para inicializar as variáveis membros, reservar memória e declarar os parâmetros do controlador.
  // Se todas as variáveis membros e alocação de memória occorrer forem inicializados corretamente, retornar CallbackReturn::SUCCESS;
  //  caso contrário, retornar CallbackReturn::ERROR.

  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  // Método on_configure: preparamos todo o resto para o controlador funcionar.
  // Em controladores, tipicamente aqui é feito a inicialização de parâmetros e set-up de interface com os componentes de hardware.

  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  //Métodos para definir as interfaces de comando/estado requeridas pelo controlador.
  // Temos três opções de configuração: ALL, NONE e INDIVIDUAL.
  //  ALL/NONE: Solicita todas/nenhuma das interfaces disponíveis, respectivamente.
  //  INDIVIDUAL: Solicita apenas as especificadas pelo controlador; requer uma lista detalhada
  //               de nomes de interfaces, tipicamente implementada por parâmetros.
  //
  // Os nomes de interface têm estrutura <nome_da_junta>/<tipo_de_interface>

  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Método on_activate: verifica e (potencialmente) organiza as interfaces requisitadas, e atribui os valores iniciais dos membros.
  //  ESSE MÉTODO É PARTE DO LAÇO DE TEMPO REAL! Logo, o código deve ser o mais curto o possível e, se possível, sem alocação de memória.
  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Método update: Laço principal do controlador.
  //  Quando esse método for chamado, as interfaces de estado têm os valores mais recentes escritos pelo hardware,
  //    e durante esse laço novos comandos para o hardware devem ser escritos nas interfaces de comando.
  //  Se formos publicar mensagens, isso é feito aqui. .
  //  ESSE MÉTODO É PARTE DO LAÇO DE TEMPO REAL! Logo, o código deve ser o mais curto o possível e, se possível, sem alocação de memória.

  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Método on_deactivate: Fazemos o oposto de on_activate. Limpamos variáveis e liberamos recursos. Também deve ser real-time safe.
  // Em muitos casos, essa função é vazia, e não precisamos implementar nada nela.

  STATE_FEEDBACK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
};

}  // namespace state_feedback_controller

#endif  // STATE_FEEDBACK_CONTROLLER__STATE_FEEDBACK_CONTROLLER_HPP_
