#include "../remoteApi/extApi.h"
#include "../include/v_repConst.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using std::placeholders::_1;

class VrepCommunication : public rclcpp::Node {
public:
    VrepCommunication() : Node("vrep_communication"), clientID(-1) {
        // Configurar la conexión
        string serverIP = "127.0.0.1";
        int serverPort = 23050;

        clientID = simxStart((simxChar*)serverIP.c_str(), serverPort, true, true, 2000, 5);
        
        if (clientID != -1) {
            RCLCPP_INFO(this->get_logger(), "Server connected!");
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&VrepCommunication::joint_callback, this, _1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Server connection problem!");
        }
    }

    ~VrepCommunication() {
        if (clientID != -1) {
            simxFinish(clientID);  // Cerrando la conexión
            RCLCPP_INFO(this->get_logger(), "Connection over!");
        }
    }

private:
    int clientID;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    int found(int a, char* b, int c, int d) {
        if (simxGetObjectHandle(a, (const simxChar*)b, (simxInt*)&c, (simxInt)simx_opmode_oneshot_wait)) {
            RCLCPP_ERROR(this->get_logger(), "No joint %d found: %s", d, b);
        } else {
            return c;
        }
        return -1;
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr data) {
        std::vector<char*> joints;
        for (const auto& name : data->name) {
            char* cstr = new char[name.length() + 1];
            strcpy(cstr, name.c_str());
            joints.push_back(cstr);
        }

        std::vector<int> joint_handle(data->name.size(), 0);
        for (size_t i = 0; i < data->name.size(); ++i) {
            joint_handle[i] = found(clientID, joints[i], joint_handle[i], i + 1);
            if (joint_handle[i] == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to find joint handle for %s", joints[i]);
            }
        }

        for (size_t i = 0; i < data->name.size(); ++i) {
            simxInt result = simxSetJointTargetPosition(clientID, (simxInt)joint_handle[i], data->position.at(i), simx_opmode_oneshot);
            if (result != simx_return_ok) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set position for joint %s. Error code: %d", joints[i], result);
            }
        }

        for (auto& joint : joints) {
            delete[] joint;
        }

        int quadrupedHandle;
        if (simxGetObjectHandle(clientID, "dummy_joint", &quadrupedHandle, simx_opmode_blocking) != simx_return_ok) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo obtener el handle del objeto 'quadruped'");
            simxFinish(clientID);
        }

        float position[3];
        if (simxGetObjectPosition(clientID, quadrupedHandle, -1, position, simx_opmode_blocking) == simx_return_ok) {
            RCLCPP_INFO(this->get_logger(), "Posición del quadruped: x = %f, y = %f, z = %f", position[0], position[1], position[2]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No se pudo obtener la posición del objeto 'quadruped'");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VrepCommunication>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
