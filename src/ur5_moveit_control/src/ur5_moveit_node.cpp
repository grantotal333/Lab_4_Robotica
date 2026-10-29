
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using JointTarget = std::map<std::string, double>;

class PickPlaceIO
{
public:
    explicit PickPlaceIO(
        const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        gripper_pub_ = node_->create_publisher<
            std_msgs::msg::Float64MultiArray>(
                "/gripper_controller/commands", 10);

        status_pub_ = node_->create_publisher<
            std_msgs::msg::String>(
                "/pick_place/status", 10);

        // Mantener la ultima salida para nuevos suscriptores.
        auto output_qos = rclcpp::QoS(1).reliable()
                                          .transient_local();

        digital_output_pub_ = node_->create_publisher<
            std_msgs::msg::Bool>(
                "/pick_place/digital_output", output_qos);

        digital_input_sub_ = node_->create_subscription<
            std_msgs::msg::Bool>(
                "/pick_place/digital_input",
                10,
                [this](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    if (msg->data && waiting_for_input_.load())
                    {
                        digital_input_received_.store(true);
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "Entrada digital TRUE recibida");
                    }
                });

        setDigitalOutput(false);
    }

    void publishStatus(const std::string& message)
    {
        std_msgs::msg::String msg;
        msg.data = message;
        status_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(), "%s", message.c_str());
    }

    void setDigitalOutput(bool value)
    {
        std_msgs::msg::Bool msg;
        msg.data = value;
        digital_output_pub_->publish(msg);
    }

    void moveGripper(bool open)
    {
        std_msgs::msg::Float64MultiArray msg;

        if (open)
        {
            msg.data = {0.04, 0.04};
            publishStatus("Abriendo gripper");
        }
        else
        {
            msg.data = {0.004, 0.004};
            publishStatus("Cerrando gripper");
        }

        gripper_pub_->publish(msg);

        // Espera de asentamiento del controlador.
        // No confirma contacto ni fuerza de agarre.
        std::this_thread::sleep_for(2s);
    }

    bool waitForDigitalInput()
    {
        digital_input_received_.store(false);
        waiting_for_input_.store(true);

        publishStatus(
            "TRANSFER completado. Esperando entrada digital TRUE");

        // Espera hasta 120 segundos de tiempo real.
        auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok())
        {
            if (digital_input_received_.load())
            {
                waiting_for_input_.store(false);
                publishStatus("Entrada digital confirmada");
                return true;
            }

            if (std::chrono::steady_clock::now() - start > 120s)
            {
                break;
            }

            std::this_thread::sleep_for(100ms);
        }

        waiting_for_input_.store(false);
        publishStatus(
            "Error: entrada digital no recibida. Rutina detenida");
        return false;
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;

    rclcpp::Publisher<
        std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Bool>::SharedPtr digital_output_pub_;

    rclcpp::Subscription<
        std_msgs::msg::Bool>::SharedPtr digital_input_sub_;

    std::atomic<bool> waiting_for_input_{false};
    std::atomic<bool> digital_input_received_{false};
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared(
        "ur5_moveit_node", options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    bool success = true;

    {
        PickPlaceIO io(node);

        try
        {
            MoveGroupInterface move_group(
                node, "manipulator");

            move_group.setPlanningPipelineId("ompl");
            move_group.setPlanningTime(10.0);
            move_group.setNumPlanningAttempts(10);
            move_group.setMaxVelocityScalingFactor(0.10);
            move_group.setMaxAccelerationScalingFactor(0.10);

            // Configuraciones articulares en radianes.
            // Validar su trayectoria respecto al entorno.
            const JointTarget HOME = {
                {"shoulder_pan_joint", 0.0},
                {"shoulder_lift_joint", -1.57},
                {"elbow_joint", 1.57},
                {"wrist_1_joint", -1.57},
                {"wrist_2_joint", -1.57},
                {"wrist_3_joint", 0.0}
            };

            const JointTarget PICK = {
                {"shoulder_pan_joint", 0.5},
                {"shoulder_lift_joint", -1.2},
                {"elbow_joint", 1.3},
                {"wrist_1_joint", -1.5},
                {"wrist_2_joint", -1.57},
                {"wrist_3_joint", 0.5}
            };

            const JointTarget TRANSFER = {
                {"shoulder_pan_joint", 0.0},
                {"shoulder_lift_joint", -1.3},
                {"elbow_joint", 1.4},
                {"wrist_1_joint", -1.5},
                {"wrist_2_joint", -1.57},
                {"wrist_3_joint", 0.0}
            };

            const JointTarget PLACE = {
                {"shoulder_pan_joint", -0.5},
                {"shoulder_lift_joint", -1.2},
                {"elbow_joint", 1.3},
                {"wrist_1_joint", -1.5},
                {"wrist_2_joint", -1.57},
                {"wrist_3_joint", -0.5}
            };

            auto moveTo = [&](
                const std::string& name,
                const JointTarget& target) -> bool
            {
                io.publishStatus("Planificando hacia " + name);

                if (!move_group.getCurrentState(10.0))
                {
                    io.publishStatus(
                        "Error: sin estado articular actual");
                    return false;
                }

                move_group.setStartStateToCurrentState();

                if (!move_group.setJointValueTarget(target))
                {
                    io.publishStatus(
                        "Objetivo articular invalido: " + name);
                    return false;
                }

                MoveGroupInterface::Plan plan;

                if (move_group.plan(plan) !=
                    moveit::core::MoveItErrorCode::SUCCESS)
                {
                    io.publishStatus(
                        "Error de planificacion: " + name);
                    return false;
                }

                io.publishStatus(
                    "Planificacion exitosa: " + name);

                if (move_group.execute(plan) !=
                    moveit::core::MoveItErrorCode::SUCCESS)
                {
                    io.publishStatus(
                        "Error de ejecucion: " + name);
                    return false;
                }

                io.publishStatus(
                    "Movimiento completado: " + name);
                return true;
            };

            io.publishStatus(
                "Iniciando Pick and Place con MoveIt + Gazebo");

            io.setDigitalOutput(false);

            // 1. HOME
            if (success) success = moveTo("HOME", HOME);
            if (success) io.moveGripper(true);

            // 2. PICK
            if (success) success = moveTo("PICK", PICK);
            if (success) io.moveGripper(false);

            // 3. TRANSFER
            if (success) success = moveTo("TRANSFER", TRANSFER);
            if (success) success = io.waitForDigitalInput();

            // 4. PLACE
            if (success) success = moveTo("PLACE", PLACE);

            if (success)
            {
                io.moveGripper(true);
                io.setDigitalOutput(true);
                io.publishStatus(
                    "PLACE completado. Salida digital TRUE");
            }

            // 5. HOME
            if (success) success = moveTo("HOME", HOME);

            if (success)
            {
                io.publishStatus(
                    "Rutina Pick and Place finalizada correctamente");
            }
            else
            {
                io.publishStatus(
                    "Rutina detenida por error");
            }

            // Mantener brevemente el nodo para entregar
            // los ultimos mensajes.
            std::this_thread::sleep_for(1s);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Excepcion: %s", e.what());
            success = false;
        }
    }

    executor.cancel();

    if (spinner.joinable())
    {
        spinner.join();
    }

    rclcpp::shutdown();

    return success ? 0 : 1;
}
