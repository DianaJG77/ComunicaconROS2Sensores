#include <rclcpp/rclcpp.hpp>
#include <exo_interfaces/srv/config_hardware_trajectory.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>
#include <vector>

class ConfigHardwareTrajectoryService : public rclcpp::Node
{
public:
    ConfigHardwareTrajectoryService() : Node("control_node_server")
    {
        service_ = create_service<exo_interfaces::srv::ConfigHardwareTrajectory>(
            "configuration", std::bind(&ConfigHardwareTrajectoryService::configuration_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Ready to configure hardware and trajectory parameters");
        Subscriber_Initialize();
    }

private:
    void configuration_callback(const std::shared_ptr<exo_interfaces::srv::ConfigHardwareTrajectory::Request> request, std::shared_ptr<exo_interfaces::srv::ConfigHardwareTrajectory_Response> response)
    {
        if (all_values_present(request))
        {
            response->success = true;
            log_configuration(request);
        }
        else
        {
            response->success = false;
            RCLCPP_INFO(get_logger(), "Failed to read parameters. Some parameters are missing");
        }
    }

    bool all_values_present(const std::shared_ptr<exo_interfaces::srv::ConfigHardwareTrajectory::Request> request)
    {
        return !request->exo_port.empty() &&
            !request->elevation_port.empty() &&
            request->sampling_frequency >= 0 &&
            request->right_knee_can_id >= 0 &&
            request->left_knee_can_id >= 0 &&
            request->right_hip_can_id >= 0 &&
            request->left_hip_can_id >= 0 &&
            request->hip_trajectory_vector.size() == static_cast<std::vector<double>::size_type>(request->size) &&
            request->knee_trajectory_vector.size() == static_cast<std::vector<double>::size_type>(request->size);
    }

    void log_configuration(const std::shared_ptr<exo_interfaces::srv::ConfigHardwareTrajectory::Request> request)
    {
        RCLCPP_INFO(get_logger(), "The parameters are being read correctly");
        if (request->verbose)
        {
            // Hardware
            RCLCPP_INFO(get_logger(), "Verbose: %s", request->verbose ? "true" : "false");
            RCLCPP_INFO(get_logger(), "Exo_port: %s", request->exo_port.c_str());
            RCLCPP_INFO(get_logger(), "Elevation_port: %s", request->elevation_port.c_str());
            RCLCPP_INFO(get_logger(), "Sampling Frequency: %ld", request->sampling_frequency);
            RCLCPP_INFO(get_logger(), "right_knee_can_id: %ld", request->right_knee_can_id);
            RCLCPP_INFO(get_logger(), "right_knee_potentiometer: %s", request->right_knee_potentiometer ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_knee_gauge: %s", request->right_knee_gauge ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_knee_fsr1: %s", request->right_knee_fsr1 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_knee_fsr2: %s", request->right_knee_fsr2 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_knee_can_id: %ld", request->left_knee_can_id);
            RCLCPP_INFO(get_logger(), "left_knee_potentiometer: %s", request->left_knee_potentiometer ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_knee_gauge: %s", request->left_knee_gauge ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_knee_fsr1: %s", request->left_knee_fsr1 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_knee_fsr2: %s", request->left_knee_fsr2 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_hip_can_id: %ld", request->right_hip_can_id);
            RCLCPP_INFO(get_logger(), "right_hip_potentiometer: %s", request->right_hip_potentiometer ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_hip_gauge: %s", request->right_hip_gauge ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_hip_fsr1: %s", request->right_hip_fsr1 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "right_hip_fsr2: %s", request->right_hip_fsr2 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_hip_can_id: %ld", request->left_hip_can_id);
            RCLCPP_INFO(get_logger(), "left_hip_potentiometer: %s", request->left_hip_potentiometer ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_hip_gauge: %s", request->left_hip_gauge ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_hip_fsr1: %s", request->left_hip_fsr1 ? "true" : "false");
            RCLCPP_INFO(get_logger(), "left_hip_fsr2: %s", request->left_hip_fsr2 ? "true" : "false");


            // Trajectory
            auto size = static_cast<std::vector<double>::size_type>(request->size);
            RCLCPP_INFO(get_logger(), "Size: %zu", size);
            for (std::vector<double>::size_type i = 0; i < size; ++i)
            {
                RCLCPP_INFO(get_logger(), "HIP in position %zu: %f", i, request->hip_trajectory_vector[i]);
                RCLCPP_INFO(get_logger(), "KNEE in position %zu: %f", i, request->knee_trajectory_vector[i]);
            }
        }
        RCLCPP_INFO(get_logger(), "[INFO] INITIALIZATION OF PARAMETERS FINISHED!\n");
    }

    void Subscriber_Initialize()
    {
        for (const auto& joint_name : joints)
        {
            for (const auto& hw_name : hw_names)
            {
                std::string sensor_name = joint_name + "_" + hw_name;
                std::string topic_name = "PUB_" + sensor_name;

                auto callback = [this, sensor_name](const std_msgs::msg::Float64::SharedPtr msg) {
                    sensors_updates[sensor_name] = msg->data;
                    RCLCPP_INFO(this->get_logger(), "Received message on topic %s: %f", sensor_name.c_str(), sensors_updates[sensor_name]);
                };

                subscribers.push_back(this->create_subscription<std_msgs::msg::Float64>(topic_name, 100, callback));
            }
        }
    }

    std::vector<std::string> joints = { "right_knee", "left_knee", "right_hip", "left_hip" };
    std::vector<std::string> hw_names = { "potentiometer", "gauge", "fsr1", "fsr2" };
    std::map<std::string, double> sensors_updates;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscribers;
    rclcpp::Service<exo_interfaces::srv::ConfigHardwareTrajectory>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConfigHardwareTrajectoryService>());
    rclcpp::shutdown();
    return 0;
}
