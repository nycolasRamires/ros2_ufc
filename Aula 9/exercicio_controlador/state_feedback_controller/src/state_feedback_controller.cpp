
#include <sys/mman.h>

#include "state_feedback_controller/state_feedback_controller.hpp"


namespace state_feedback_controller
{

// Construtor; aqui o trabalho é feito realmente no on_init().
StateFeedbackController::StateFeedbackController(void):  q_(0), dq_(0), qr_(0), tau_(0), referencePoint_(nullptr) {}

controller_interface::CallbackReturn StateFeedbackController::on_init()
{
  // Aqui deve ter código para lidar com quaisquer erros que possamos encontrar na inicialização.
  //  A maneira preguiçosa de fazer isso (como abaixo) é colocar tudo em um laço de try-catch e jogar erro em uma exceção.
  try{
    // Parâmetros são declarados aqui.
    //   Segundo a documentação de ControllerInterface, usamos auto_declare no ros2_control, que é um wrapper para a função node->declare_parameter do ROS.
    auto_declare<std::vector<double>>("state_feedback_matrix", std::vector<double>());

    jointNames_ = auto_declare<std::vector<std::string>>("joints", jointNames_);

    auto_declare<int>("priority",sched_get_priority_max(SCHED_FIFO)); // Prioridade do processo; por default solicita máxima prioridade

    // // Inicializa pontos de interpolação da trajetória de referência da junta com zeros. 
    // point_interp_.positions.assign(jointNames_.size(), 0);
    // point_interp_.velocities.assign(jointNames_.size(), 0);
  }catch(const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown on on_init() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  // Inscrição no tópico /reference_point para obter um ponto de referência.
  // using std::placeholders::_1;
  // rclcpp::QoS qos(rclcpp::KeepLast(1));
  // qos.transient_local();
  // auto robotDescriptionSubscriber=get_node()->create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&
  //   StateFeedbackController::robotDescriptionCB,this,_1));

  // while(robotDescription_.empty())
  // {
  //   RCLCPP_WARN_SKIPFIRST_THROTTLE(get_node()->get_logger(),
  //     *get_node()->get_clock(),1000,"Waiting for robot model on /robot_description.");
  //   rclcpp::spin_some(get_node()->get_node_base_interface());
  // }

  return CallbackReturn::SUCCESS;
}

// Configuração do controlador
controller_interface::CallbackReturn StateFeedbackController::on_configure(const rclcpp_lifecycle::State &)
{
  try{
    // Inscrição   
    using std::placeholders::_1;
    sub_command_=get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "command",1,std::bind(&StateFeedbackController::commandCB,this,_1));

    // Preenche parâmetros
    jointNames_=get_node()->get_parameter("joints").as_string_array();
    if(jointNames_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(),"'joints' parameter was empty.");
      return CallbackReturn::ERROR;
    }

    q_.resize(jointNames_.size());
    dq_.resize(jointNames_.size());
    tau_.resize(jointNames_.size());

    // Preenchendo matriz de ganhos de feedback de estados
    K_.resize(jointNames_.size(), jointNames_.size()*2);
    std::vector<double> KVec=get_node()->get_parameter("state_feedback_matrix").as_double_array();
    if(KVec.empty()){
      throw std::runtime_error("No 'state_feedback_matrix' in controller!");
    }
    K_=Eigen::Map<Eigen::MatrixXd>(KVec.data(),jointNames_.size() , jointNames_.size()*2);

    // auto callback = [this](const trajectory_msgs::msg::JointTrajectory traj_msg) -> void
    // {
    //   RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory.");
    //   traj_msg_external_.set(traj_msg);
    //   new_msg_ = true;
    // };

    // joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    //     "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  }catch(const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown in on_configure() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  if(!get_node()->get_parameter("priority",priority_))
    RCLCPP_WARN(get_node()->get_logger(),"No ’priority’ configured for controller. Using highest possible priority.");

  return CallbackReturn::SUCCESS;
}


/// CONFIGURAÇÃO DAS INTERFACES:
// Ambas essas funções devem retornar um objeto InterfaceConfiguration, especificando as interfaces que o controlador solicitará.

// Configuração de interface de comando
controller_interface::InterfaceConfiguration StateFeedbackController::command_interface_configuration()
  const
{
  // Solicita as interfaces de comando por esforço conforme nomes definidos em jointNames_
  // controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  conf.names.reserve(jointNames_.size() );
  for (const auto & joint_name : jointNames_)
  {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT); // Nome da interface segue a regra <nome_da_junta>/<tipo_de_interface>
  }

  return conf;
}

// Configuração de interface de estado
controller_interface::InterfaceConfiguration StateFeedbackController::state_interface_configuration() const
{
  // Outra forma de inicializar uma estrutura InterfaceConfiguration;
  //  a linha a seguir tem o mesmo efeito que as duas primeiras linhas da função command_interface_configuration()
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(jointNames_.size() * 2); // Cada junta tem interfaces de posição e velocidade
  for (const auto & joint_name : jointNames_)
  {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return conf;
}

/// MÉTODOS DE TEMPO REAL: on_activate, on_update, on_deactivate
controller_interface::CallbackReturn StateFeedbackController::on_activate(const rclcpp_lifecycle::State &)
{
  // Limpo os vetores com as interfaces, caso tenha ocorrido um reset do controlador.
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // Atribuição das interfaces de comando...
  for (auto & interface : command_interfaces_) // Lê-se "para cada interface em command_interfaces_"
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // ... e de estado.
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // Inicializa posições de juntas e posição de referência
  for(unsigned int i=0; i < jointNames_.size(); i++)
		{
      // Obtém os valores disponíveis na hardware interface; caso contrário, mantém o valor atual
			q_(i)=state_interfaces_[2*i].get_optional().value_or(q_(i));
			dq_(i)=state_interfaces_[2*i+1].get_optional().value_or(dq_(i));
		}
		qr_=q_;

  // Configura prioridade do processo em tempo real
  struct sched_param param;
  param.sched_priority=priority_;
  if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
    RCLCPP_WARN(get_node()->get_logger(),"Failed to set real-time scheduler.");
  else if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    RCLCPP_WARN(get_node()->get_logger(),"Failed to lock memory.");

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type StateFeedbackController::update(
  const rclcpp::Time &/*time*/, const rclcpp::Duration & /*period*/) // Como não usamos nenhum dos dois argumentos aqui comentamos eles
{
  // Obtém ponto de referência mais recente
  auto referencePoint = referencePoint_.readFromRT();
  if(referencePoint && *referencePoint) // checa se o ponto de referência recebido e seu ponteiro são válidos
  {
    if((*referencePoint)->positions.size() == jointNames_.size())
      qr_ = Eigen::VectorXd::Map((*referencePoint)->positions.data(),jointNames_.size());
  }

  // Atualiza variáveis com os valores de posição e velocidade das juntas
  for(unsigned int i=0;i < jointNames_.size();i++)
  {
    // Função auxiliar get_hw_value(), definida posteriormente
    q_(i) = this->get_hw_value(state_interfaces_[2*i]);
    dq_(i) = this->get_hw_value(state_interfaces_[2*i+1]);
  }

  Eigen::VectorXd stateVec(q_.size()*2);
  RCLCPP_DEBUG(get_node()->get_logger(), "q_: %f, dq_: %f", q_(0), dq_(0));
  RCLCPP_DEBUG(get_node()->get_logger(), "stateVec.size:  %ld", stateVec.size());
  stateVec << q_, dq_; // concatena posições e velocidades

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "qr_: " << qr_);
  Eigen::VectorXd stateRefVec(q_.size()*2);
  stateRefVec << qr_, Eigen::VectorXd::Zero(dq_.size()); // referência de velocidade nula

  // Calcula esforço a ser aplicado nas juntas
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "K_:  " << K_);
  tau_ = -K_ * (stateVec - stateRefVec);
  // tau_ = -K_ * stateVec;

  // Aplica comando nas juntas

  for (unsigned int i=0; i < jointNames_.size(); i++)
  {
    auto success = command_interfaces_[i].set_value(tau_(i));
    if(!success)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command value for %s",
        command_interfaces_[i].get_name().c_str());
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn StateFeedbackController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for(unsigned int i=0; i < jointNames_.size(); i++)
    q_(i) = get_hw_value(state_interfaces_[2*i]);
  tau_.setZero();
  return CallbackReturn::SUCCESS;
}

/// FUNÇÕES DE CALLBACK PARA COMUNICAÇÃO VIA TÓPICOS
void StateFeedbackController::commandCB(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr referencePoint)
{
  referencePoint_.writeFromNonRT(referencePoint);
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Received new reference point: " << referencePoint->positions[0]);
}

/// FUNÇÕES AUXILIARES, SE NECESSÁRIAS
double StateFeedbackController::get_hw_value(const hardware_interface::LoanedStateInterface & interface, const unsigned int max_tries)
{
  const auto interface_value_op = interface.get_optional(max_tries);
  if (!interface_value_op.has_value())
  {
    RCLCPP_DEBUG(get_node()->get_logger(),
      "Unable to retrieve the state interface value for %s", interface.get_name().c_str());
    throw std::runtime_error("Could not get value from interface " + interface.get_name());
  }
  return interface_value_op.value();
}


// Função para sintonizar a matriz de ganhos de feedback de estados
void tuneLQRMatrix(Eigen::MatrixXd & K, double zeta, double wn)
{
  // K = [Kp Kd], onde Kp = wn^2*I e Kd = 2*zeta*wn*I
  unsigned int n = K.cols() / 2;
  K.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n) * wn * wn;
  K.block(0, n, n, n) = Eigen::MatrixXd::Identity(n, n) * 2.0 * zeta * wn;
}

}  // namespace state_feedback_controller

//// REGISTRO DO CONTROLADOR COMO PLUGIN

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  state_feedback_controller::StateFeedbackController, controller_interface::ControllerInterface)
