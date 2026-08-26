#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


constexpr double S = -60;
constexpr double A = 30.0;
constexpr double T = 100;


namespace configRobot {
  constexpr double L1 = 56.5;
  constexpr double L2 = 72;
  constexpr double L3 = 126.59;
  constexpr double x0 = 170;
}

namespace crabWalking {
  constexpr std::size_t dataPointPerLine = 20;
  constexpr std::size_t numDirections = 12;
  constexpr double angleResolution = 30;
}

namespace waveGait {
  constexpr std::size_t waveTotal = 120;
  constexpr std::size_t waveSwing = 20;
  constexpr std::size_t waveStance = waveTotal - waveSwing;
}

enum class State { 
  LIFT_UP, 
  HOLD, 
  LIFT_DOWN, 
  DRAG 
};

enum class RobotMode {
  CRAB_WALK,
  WAVE_WALK,
  ROTATE,
  STOP
};


struct Leg {
  std::string name;
  uint8_t channelL1, channelL2, channelL3;
  double rotang;
  double offset1, offset2, offset3;
};


/* ============================================================
   MATH / CINEMATIQUE
   ============================================================ */

double bezier(double t, double p1, double p2, double p3){
  return (1-t)*(1-t)*p1 + 2*(1-t)*t*p2 + t*t*p3;
}

bool invK(double x, double y, double z, double &J1, double &J2, double &J3) {
  double theta_1 = std::atan2(y, x);
  double r2 = x / std::cos(theta_1) - configRobot::L1;
  double r1 = std::sqrt(z*z + r2*r2);

  double val2 = (configRobot::L2*configRobot::L2 + r1*r1 - configRobot::L3*configRobot::L3) / (2*configRobot::L2*r1);
  double val3 = (configRobot::L2 * configRobot::L2 + configRobot::L3 * configRobot::L3 - r1 * r1) / (2.0 * configRobot::L2 * configRobot::L3);
  if (val2 < -1.0 || val2 > 1.0 || val3 < -1.0 || val3 > 1.0) return false;

  double theta_2 = std::acos(val2) + std::atan2(z, r2);
  double theta_3 = M_PI - std::acos(val3);

  J1 = theta_1 * (180.0 / M_PI) + 90.0;
  J2 = theta_2 * (180.0 / M_PI) + 90.0;
  J3 = theta_3 * (180.0 / M_PI);
  return true;
}

void matrixGlobalToLocal(double xGlobal, double yGlobal, double angleRot, double &xLocal, double &yLocal){
    xLocal = xGlobal * std::cos(angleRot) - yGlobal * std::sin(angleRot) + configRobot::x0;
    yLocal = xGlobal * std::sin(angleRot) + yGlobal * std::cos(angleRot);
}

class hexapodGaitNode : public rclcpp::Node{
public:
    hexapodGaitNode() : Node("hexapod_gait_node"){
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        legs_[0] = {"leg1", 0, 1, 2,   0.0,  15.0,  5.0, -10.0};
        legs_[1] = {"leg2", 3, 4, 5, -60.0,   3.0, -5.0,  -2.0};
        legs_[2] = {"leg3", 6, 7, 8, -120.0, -2.0, -5.0, -15.0};
        legs_[3] = {"leg4", 9, 10, 11, 180.0, 12.0,  5.0,   0.0};
        legs_[4] = {"leg5", 12, 13, 14, 120.0, -2.0,  4.0, -20.0};
        legs_[5] = {"leg6", 15, 16, 17,  60.0, -2.0,  4.0, -20.0};

        initJointNames();

        linspace();
        wave_linspace();
        generate_crab_LUT();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40);
            std::bind(&hexapodGaitNode::controlLoop, this);
        );

        RCLCPP_INFO(this->get_logger(), "Nœud HexapodGaitNode initialisé avec succès.");
    }
private:
    double swingY[crabWalking::dataPointPerLine];
    double swingZ[crabWalking::dataPointPerLine];
    double stanceY[crabWalking::dataPointPerLine];
    double stanceZ[crabWalking::dataPointPerLine];

    double wave_swingY[waveGait::waveSwing];
    double wave_swingZ[waveGait::waveSwing];
    double wave_stanceY[waveGait::waveStance];
    double wave_stanceZ[waveGait::waveStance];

    double crab_LUT[crabWalking::numDirections][6][40][3];
    Leg legs_[6];
    RobotMode mode_{RobotMode::STOP};
    std::size_t dir_index_{0};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;

    void initJointNames() {
        for (std::size_t i = 1; i < 7; ++i) {
            joint_names_.push_back("leg" + std::to_string(i) + "_joint1");
            joint_names_.push_back("leg" + std::to_string(i) + "_joint2");
            joint_names_.push_back("leg" + std::to_string(i) + "_joint3");
        }
    }

    void linspace(){
      for (std::size_t k = 0; k < crabWalking::dataPointPerLine; k++){ /*The variable k is here to fill each ...[k] with 20 points*/
        double t = k / static_cast<double>(crabWalking::dataPointPerLine - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
        swingY_[k] = bezier(t, -T/2.0, 0.0, T/2.0);
        swingZ_[k] = bezier(t, S, S + 2.0*A, S);
        stanceY_[k] = T/2.0 - t * T;
        stanceZ_[k] = S;
      }
    }

    void wave_linspace(){
      for (std::size_t k = 0; k < waveGait::waveSwing; k++){ /*The variable k is here to fill each ...[k] with 20 points*/
        double t = k / static_cast<double>(waveGait::waveSwing - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
        wave_swingY_[k] = bezier(t, -T/2.0, 0.0, T/2.0);
        wave_swingZ_[k] = bezier(t, S, S + 2.0*A, S);
      }
      for (std::size_t k = 0; k < waveGait::waveStance; k++){ /*The variable k is here to fill each ...[k] with 100 points*/
        double t = k / static_cast<double>(waveGait::waveStance - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
        wave_stanceY_[k] = T/2.0 - t * T;
        wave_stanceZ_[k] = S;
      }
    }

    double getY(std::size_t index){
      std::size_t idx = index % 40; /*Make the input repeat every 40 "step" (see l-196)*/
      return (idx < 20) ? swingY_[idx] : stanceY_[idx - 20]; /*if idx is between 0-19, return swingY[idx] and if between 20-39, return stanceY[idx]*/
    }

    double getZ(std::size_t index){
      std::size_t idx = index % 40; /*Make the input repeat every 40 "steps" (see l-197)*/
      return (idx < 20) ? swingZ_[idx] : stanceZ_[idx - 20]; /*if idx is between 0-19, return swingZ[idx] and if between 20-39, return stanceZ[idx]*/
    }

    void generate_crab_LUT() {
      for (std::size_t d = 0; d < crabWalking::numDirections; d++){ 
        double dir_angle = d * crabWalking::angleResolution; /*dir_angle can take 12 angles : 0°, 30°, 60°,..., 300°, 330°. 360° is equal to 0° so no need to write it*/
        for (std::size_t j = 0; j < 6; j++){ /*j represent each leg : legs[0], legs[1]...*/
          for (std::size_t step = 0; step < 40; step++){ /*In each LUT for each angle so 0° to 330° (or each direction from 0 to 11), we divid the walking cycle to 40 points or "step" (from 0 to 39).*/
            double y_raw = getY(step); /*Get the desired Y position in the basic walking cycle(the origin leg in the forward walking cycle)*/
            double z_raw = getZ(step); /*Get the desired Z position in the basic walking cycle(the origin leg in the forward walking cycle)*/
            double rot = (legs[j].rotang + dir_angle) * (M_PI / 180.0); /*Take the leg's orientation angle and adds the current walking direction angle. Converts the total to radians.*/
            double x = x0 - y_raw * std::sin(rot); /*rotation matrix multiplied by matrix of raw position (x_raw  y_raw z_raw) + matrix of offsets but the deplacment in x is equal to 0*/
            double y = y_raw * std::cos(rot);
            double z = z_raw;

            double J1, J2, J3;
            if (invK(x, y, z, J1, J2, J3)) { /*Meaning if the position calculated by invK can be reached, the function return true and fill J1, J2 and J3.*/
              crab_LUT[d][j][step][0] = angleToTick(J1 + legs[j].offset1); /*J1, J2 and J3 are converted into radians and the result is stocked into the CRAB_LUT*/
              crab_LUT[d][j][step][1] = angleToTick(J2 + legs[j].offset2);
              crab_LUT[d][j][step][2] = angleToTick(J3 + legs[j].offset3);
            } else if (step > 0) { /*If the position can't be reached by the leg, we stock the previous position*/
              crab_LUT[d][j][step][0] = crab_LUT[d][j][step-1][0];
              crab_LUT[d][j][step][1] = crab_LUT[d][j][step-1][1];
              crab_LUT[d][j][step][2] = crab_LUT[d][j][step-1][2];
            }
          }
        }
      }
    }
}