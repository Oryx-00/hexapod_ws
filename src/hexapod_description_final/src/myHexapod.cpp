#include <chrono>
#include <memory>
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <cstdint>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

double degToRad(double angle){
    return angle * (M_PI / 180.0);
}

double radToDeg(double angle){
    return angle * (180.0 / M_PI);
}

constexpr double S = 0.0;
constexpr double A = 30.0;
constexpr double T = 100.0;
constexpr double distanceFromCoxa = 170.0;
constexpr double heightFromCoxa = -60.0;
constexpr std::size_t pointPerTrajectory = 20;

namespace configRobot {
  constexpr double L1 = 56.5;
  constexpr double L2 = 72.0;
  constexpr double L3 = 126.59;
}


struct vector3D {
    double x, y, z;
};

struct Leg {
    std::string name;
    uint8_t channelL1, channelL2, channelL3;
    double rotang;
    double offsetAng1, offsetAng2, offsetAng3;
    double bodyCoxaDistance;

    vector3D coxaOffset;
    vector3D homePos;

    Leg(std::string name, uint8_t ch1, uint8_t ch2, uint8_t ch3,
        double rotang, double off1, double off2, double off3, double bodyCoxaDist)
        : name(name), channelL1(ch1), channelL2(ch2), channelL3(ch3), rotang(rotang)
        , offsetAng1(off1), offsetAng2(off2), offsetAng3(off3), bodyCoxaDistance(bodyCoxaDist)
    {
        double rad = degToRad(rotang);
        coxaOffset = {bodyCoxaDist * std::cos(rad), bodyCoxaDist * std::sin(rad), 0.0};
        homePos = {
            coxaOffset.x + distanceFromCoxa * std::cos(rad),
            coxaOffset.y + distanceFromCoxa * std::sin(rad),
            coxaOffset.z + heightFromCoxa
        };
    }
};

enum class RobotMode {
  CRAB_WALK,
  WAVE_WALK,
  ROTATE,
  STOP
};

class HexapodGaitNode : public rclcpp::Node {
public:
    HexapodGaitNode() : Node("hexapod_gait_node"), idx_(0), mode_actuel_(RobotMode::CRAB_WALK){

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        initJointNames();
        linspaceCrab();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&HexapodGaitNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Nœud HexapodGaitNode initialisé avec succès.");
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;

    std::array<double, pointPerTrajectory> swingY_;
    std::array<double, pointPerTrajectory> swingZ_;
    std::array<double, pointPerTrajectory> stanceY_;
    std::array<double, pointPerTrajectory> stanceZ_;

    std::size_t idx_;
    RobotMode mode_actuel_;

    const std::array<Leg, 6> legs = {{
        {"leg1", 0,  1,  2, 0.0, 0.0, 0.0, 0.0, 100.0},
        {"leg2", 3,  4,  5,  60.0, 0.0, 0.0, 0.0, 120.0},
        {"leg3", 6,  7,  8, 120.0, 0.0, 0.0, 0.0, 120.0},
        {"leg4", 9, 10, 11,  180.0, 0.0, 0.0, 0.0, 100.0},
        {"leg5", 12, 13, 14, -120.0, 0.0, 0.0, 0.0, 120.0},
        {"leg6", 15, 16, 17,  -60.0, 0.0,  0.0, 0.0, 120.0}
    }};


    double generateBezierCurve(double t, double p0, double p1, double p2){
        return (1.0 - t) * (1.0 - t) * p0 + 2.0 * (1.0 - t) * t * p1 + t * t * p2;
    }

    void initJointNames() {
        joint_names_.clear();
        for (std::size_t i = 1; i <= 6; ++i) {
            joint_names_.push_back("leg" + std::to_string(i) + "_joint1");
            joint_names_.push_back("leg" + std::to_string(i) + "_joint2");
            joint_names_.push_back("leg" + std::to_string(i) + "_joint3");
        }
    }

    void linspaceCrab(){
        for (std::size_t k = 0; k < pointPerTrajectory; k++){
            double t = k / static_cast<double>(pointPerTrajectory - 1.0);
            swingY_[k] = generateBezierCurve(t, -T/2.0, 0.0, T/2.0);
            swingZ_[k] = generateBezierCurve(t, S, S + 2.0 * A, S);
            stanceY_[k] = T/2.0 - t * T;
            stanceZ_[k] = S;
        }
    }

    double yTrajectory(std::size_t index){
        if (index < 20){
            return swingY_[index];
        } else {
            return stanceY_[index - 20];
        }
    }

    double zTrajectory(std::size_t index){
        if (index < 20){
            return swingZ_[index];
        } else {
            return stanceZ_[index - 20];
        }
    }

    void invKinematics(double x, double y, double z, double &J1, double &J2, double &J3){
        double theta_1 = std::atan2(y, x);
        double r2 = std::sqrt(x * x + y * y) - configRobot::L1;
        double r1 = std::sqrt(z * z + r2 * r2);

        double val2 = (configRobot::L2 * configRobot::L2 + r1 * r1 - configRobot::L3 * configRobot::L3) / (2.0 * configRobot::L2 * r1);
        double val3 = (configRobot::L2 * configRobot::L2 + configRobot::L3 * configRobot::L3 - r1 * r1) / (2.0 * configRobot::L2 * configRobot::L3);

        val2 = std::clamp(val2, -1.0, 1.0);
        val3 = std::clamp(val3, -1.0, 1.0);

        double theta_2 = std::acos(val2) + std::atan2(z, r2);
        double theta_3 = M_PI - std::acos(val3);

        J1 = radToDeg(theta_1);
        J2 = radToDeg(theta_2);
        J3 = -radToDeg(theta_3);
    }

    void crabGaitStep(std::size_t step, sensor_msgs::msg::JointState & msg){
        for (std::size_t j = 0; j < 6; j++){
            std::size_t trajIdx = (j % 2 == 0) ? step : (step + 20) % 40;

            double xBody = legs[j].homePos.x;
            double yBody = legs[j].homePos.y + yTrajectory(trajIdx);
            double zBody = legs[j].homePos.z + zTrajectory(trajIdx);

            double xRel = xBody - legs[j].coxaOffset.x;
            double yRel = yBody - legs[j].coxaOffset.y;
            double zRel = zBody;

            double rad = degToRad(legs[j].rotang);
            double xLocal =  xRel * std::cos(rad) + yRel * std::sin(rad);
            double yLocal = -xRel * std::sin(rad) + yRel * std::cos(rad);
            double zLocal =  zRel;

            double J1, J2, J3;
            invKinematics(xLocal, yLocal, zLocal, J1, J2, J3);

            J1 += legs[j].offsetAng1;
            J2 += legs[j].offsetAng2;
            J3 += legs[j].offsetAng3;

            msg.position.push_back(degToRad(J1));
            msg.position.push_back(degToRad(J2));
            msg.position.push_back(degToRad(J3));
        }
    }


    void controlLoop(){
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        switch(mode_actuel_){
            case RobotMode::CRAB_WALK:
                crabGaitStep(idx_, msg);
                break;

            case RobotMode::WAVE_WALK:
                break;
            
            case RobotMode::ROTATE:
                break;
            
            case RobotMode::STOP:
                break;
        }
        joint_pub_->publish(msg);
        idx_ = (idx_ + 1) % 40;
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodGaitNode>());
    rclcpp::shutdown();
    return 0;
}