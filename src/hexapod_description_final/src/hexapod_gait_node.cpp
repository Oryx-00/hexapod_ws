#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

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
RobotMode mode = RobotMode::STOP;

struct Leg {
  std::string name;
  int channelL1, channelL2, channelL3;
  float rotang;
  float offset1, offset2, offset3;
};
Leg legs[6];


namespace rotationGait {
  double lastY[6] = {0, 0, 0, 0, 0, 0};
  double lastZ[6] = {S, S, S, S, S, S};
  double turn_direction = 1.0;
  std::size_t blockIndex = 0;
  std::size_t stepIndex = 0;
  std::size_t stepsPerBlock = 20;
  double drag_amplitude = 60.0;
  double liftHeight = 60.0;
}




double swingY[crabWalking::dataPointPerLine];
double swingZ[crabWalking::dataPointPerLine];
double stanceY[crabWalking::dataPointPerLine];
double stanceZ[crabWalking::dataPointPerLine];

double wave_swingY[waveGait::waveSwing];
double wave_swingZ[waveGait::waveSwing];
double wave_stanceY[waveGait::waveStance];
double wave_stanceZ[waveGait::waveStance];

constexpr std::size_t wave_order[6] = {1, 0, 5, 4, 3, 2}; /*The order that the hexapod follow to know which leg to move*/





int crab_LUT[crabWalking::numDirections][6][40][3];
std::size_t dir_index = 0;

State sequenceA[6] = {State::LIFT_UP, State::HOLD, State::LIFT_DOWN, State::HOLD, State::DRAG, State::HOLD};
State sequenceB[6] = {State::HOLD, State::DRAG, State::HOLD, State::LIFT_UP, State::HOLD, State::LIFT_DOWN};




/* ============================================================
   MATH / CINEMATIQUE
   ============================================================ */

double bezier(double t, double p1, double p2, double p3){
  return (1-t)*(1-t)*p1 + 2*(1-t)*t*p2 + t*t*p3;
}

int angleToTick(double angle){
  double ratio = angle / 180.0;
  return static_cast<int>(ratio * static_cast<double>(configRobot::servoMax - configRobot::servoMin) + static_cast<double>(configRobot::servoMin));
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

void move_joints(double J1, double J2, double J3, Leg currentLeg){
  uint16_t tick1 = angleToTick(J1 + currentLeg.offset1);
  uint16_t tick2 = angleToTick(J2 + currentLeg.offset2);
  uint16_t tick3 = angleToTick(J3 + currentLeg.offset3);

  if (currentLeg.pwm_id == 1){
    pwm1.setPWM(currentLeg.channelL1, 0, tick1);
    pwm1.setPWM(currentLeg.channelL2, 0, tick2);
    pwm1.setPWM(currentLeg.channelL3, 0, tick3);
  } else {
    pwm2.setPWM(currentLeg.channelL1, 0, tick1);
    pwm2.setPWM(currentLeg.channelL2, 0, tick2);
    pwm2.setPWM(currentLeg.channelL3, 0, tick3);
  }
}

void move_xyz(double x, double y, double z, Leg currentLeg) {
  double J1, J2, J3;
  if (invK(x, y, z, J1, J2, J3)) {
    move_joints(J1, J2, J3, currentLeg);
  }
}


/* ============================================================
   TRAJECTOIRES + LUT
   ============================================================ */

void linspace(){
  for (std::size_t k = 0; k < crabWalking::dataPointPerLine; k++){ /*The variable k is here to fill each ...[k] with 20 points*/
    double t = k / static_cast<double>(crabWalking::dataPointPerLine - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
    swingY[k] = bezier(t, -T/2, 0.0, T/2);
    swingZ[k] = bezier(t, S, S + 2*A, S);
    stanceY[k] = T/2 - t * T;
    stanceZ[k] = S;
  }
}

void wave_linspace(){
  for (std::size_t k = 0; k < waveGait::waveSwing; k++){ /*The variable k is here to fill each ...[k] with 20 points*/
    double t = k / static_cast<double>(waveGait::waveSwing - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
    wave_swingY[k] = bezier(t, -T/2, 0.0, T/2);
    wave_swingZ[k] = bezier(t, S, S + 2*A, S);
  }
  for (std::size_t k = 0; k < waveGait::waveStance; k++){ /*The variable k is here to fill each ...[k] with 100 points*/
    double t = k / static_cast<double>(waveGait::waveStance - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
    wave_stanceY[k] = T/2 - t * T;
    wave_stanceZ[k] = S;
  }
}


double getY(std::size_t index){
  std::size_t idx = index % 40; /*Make the input repeat every 40 "step" (see l-196)*/
  return (idx < 20) ? swingY[idx] : stanceY[idx - 20]; /*if idx is between 0-19, return swingY[idx] and if between 20-39, return stanceY[idx]*/
}

double getZ(std::size_t index){
  std::size_t idx = index % 40; /*Make the input repeat every 40 "steps" (see l-197)*/
  return (idx < 20) ? swingZ[idx] : stanceZ[idx - 20]; /*if idx is between 0-19, return swingZ[idx] and if between 20-39, return stanceZ[idx]*/
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


/* ============================================================
   GAITS
   ============================================================ */

void crab_walk(){
  for (std::size_t j = 0; j < 6; j++) {
    std::size_t legIndex = (j % 2 == 0) ? crab_walk::i : (crab_walk::i + 20) % 40; /*if even leg (j = 0, 2, 4), legIndex = i and if odd leg (j = 1, 3, 5), legIndex is shifted by 20 step out of 40*/
    uint16_t tickL1 = crab_LUT[dir_index][j][legIndex][0]; /*dir_index stand for the direction of the crab_walk, j for the leg, legIndex for the step (from 0 to 39) and 0, 1 or 2 for the coxa femur and tibia.*/
    uint16_t tickL2 = crab_LUT[dir_index][j][legIndex][1];
    uint16_t tickL3 = crab_LUT[dir_index][j][legIndex][2];

    if (legs[j].pwm_id == 1) { /*if the leg'servos are connected to the first pca, then send send the ticks to the first pca*/
      pwm1.setPWM(legs[j].channelL1, 0, tickL1);
      pwm1.setPWM(legs[j].channelL2, 0, tickL2);
      pwm1.setPWM(legs[j].channelL3, 0, tickL3);
    } else { /*if the leg'servos are connected to the second pca, then send send the ticks to the second pca*/
      pwm2.setPWM(legs[j].channelL1, 0, tickL1);
      pwm2.setPWM(legs[j].channelL2, 0, tickL2);
      pwm2.setPWM(legs[j].channelL3, 0, tickL3);
    }
  }
  crab_walk::i++; /*Increment i each time to fill the CRAB_LUT with the position of the next step*/
  if (crab_walk::i >= 40) crab_walk::i = 0;
}

void wave_gait(){
  for (std::size_t pos = 0; pos < 6; pos++){
    std::size_t j = wave_order[pos]; /*As usual, j represent the leg*/
    std::size_t localIdx = (wave_gait::wave_i + waveGait::waveTotal - pos * (waveGait::waveTotal / 6)) % waveGait::waveTotal; /*Every wave_i += 20 (so 20 steps), one of the leg start the swinging phase for 20 steps while the others are doing their stance phase for 100 steps*/

    double y, z;
    if (localIdx < waveGait::waveSwing){ /*if localIdx < 20, which means in the swing phase*/
      y = wave_swingY[localIdx]; /*Get the desired Y position in the table*/
      z = wave_swingZ[localIdx]; /*Get the desired Z position in the table*/
    } else { /*if localIdx > 20, which means in the stance phase*/
      y = wave_stanceY[localIdx - waveGait::waveSwing]; /*The substaction is here to remove the waveSwing'value (20) and prevent overflow*/
      z = wave_stanceZ[localIdx - waveGait::waveSwing];
    }

    double rot = legs[j].rotang * (M_PI / 180.0); /* Same as generate_crab_LUT(). See l-198*/
    double x = x0 - y * std::sin(rot);
    double y_final = y * std::cos(rot);
    move_xyz(x, y_final, z, legs[j]);
  }
  crab_walk::wave_i++;
  if (crab_walk::wave_i >= waveGait::waveTotal) waveGait::wave_i = 0;
}

void move_leg_rotation(Leg leg, State state, std::size_t step, std::size_t legIndex){
  double t = step / static_cast<double>(rotationGait::stepsPerBlock - 1); /*The variable t takes value from 0.0 to 1.0 equally spaced*/
  double x = x0; /*x is fixed in the local coordinate system of the leg*/
  double y = rotationGait::lastY[legIndex]; /*Initialize the position in the HOLD phase*/
  double z = rotationGait::lastZ[legIndex];

  switch (state) {
    case State::HOLD: break;

    case State::LIFT_UP:
      z = S + t * rotationGait::liftHeight; /* move in diagonal*/
      y = t * rotationGait::drag_amplitude * rotationGait::turn_direction;
      break;

    case State::LIFT_DOWN: /* move verticaly*/
      z = S + (1.0 - t) * rotationGait::liftHeight;
      y = rotationGait::drag_amplitude * rotationGait::turn_direction;
      break;
      
    case State::DRAG: /* move horizontaly to the initial position*/
      z = S;
      y = rotationGait::drag_amplitude * rotationGait::turn_direction * (1.0 - t);
      break;
  }


  rotationGait::lastY[legIndex] = y; /*Store the current position so that the robot knows where it is*/
  rotationGait::lastZ[legIndex] = z;
  move_xyz(x, y, z, leg);
}

void rotate(){
  for (std::size_t j = 0; j < 6; j++) { /*As usual, j represent the leg*/
    State state = (j % 2 == 0) ? sequenceA[rotationGait::blockIndex] : sequenceB[rotationGait::blockIndex];/*if j is even, state selects the sequenceA for the leg j. blockIndex is initialy equal to 0*/
    move_leg_rotation(legs[j], state, rotationGait::stepIndex, j);
  }
  rotationGait::stepIndex++;
  if (rotationGait::stepIndex >= rotationGait::stepsPerBlock){ /*if stepIndex is > 20*/
    rotationGait::stepIndex = 0;
    rotationGait::blockIndex = (rotationGait::blockIndex + 1) % 6; /*blockIndex selets the next movement (see l-83)*/
  }
}


/* ============================================================
   SETUP
   ============================================================ */

void setup() {
  Serial.begin(115200);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  legs[0].channelL1 = 0;  legs[0].channelL2 = 1;  legs[0].channelL3 = 2;
  legs[0].rotang = 0;     legs[0].pwm_id = 1;     legs[0].C = 100;
  legs[0].offset1 = 15;   legs[0].offset2 = 5;    legs[0].offset3 = -10;

  legs[1].channelL1 = 3;  legs[1].channelL2 = 4;  legs[1].channelL3 = 5;
  legs[1].rotang = -60;   legs[1].pwm_id = 1;     legs[1].C = 120;
  legs[1].offset1 = 3;    legs[1].offset2 = -5;   legs[1].offset3 = -2;

  legs[2].channelL1 = 6;  legs[2].channelL2 = 7;  legs[2].channelL3 = 8;
  legs[2].rotang = -120;  legs[2].pwm_id = 1;     legs[2].C = 120;
  legs[2].offset1 = -2;   legs[2].offset2 = -5;   legs[2].offset3 = -15;

  legs[3].channelL1 = 9;  legs[3].channelL2 = 10; legs[3].channelL3 = 11;
  legs[3].rotang = 180;   legs[3].pwm_id = 1;     legs[3].C = 100;
  legs[3].offset1 = 12;   legs[3].offset2 = 5;    legs[3].offset3 = 0;

  legs[4].channelL1 = 12; legs[4].channelL2 = 13; legs[4].channelL3 = 14;
  legs[4].rotang = 120;   legs[4].pwm_id = 1;     legs[4].C = 120;
  legs[4].offset1 = -2;   legs[4].offset2 = 4;    legs[4].offset3 = -20;

  legs[5].channelL1 = 0;  legs[5].channelL2 = 1;  legs[5].channelL3 = 2;
  legs[5].rotang = 60;    legs[5].pwm_id = 2;     legs[5].C = 120;
  legs[5].offset1 = -2;   legs[5].offset2 = 4;    legs[5].offset3 = -20;

  

  linspace();
  wave_linspace();
  generate_crab_LUT();

}


/* ============================================================
   LOOP
   ============================================================ */

void loop() {
  switch(mode){
    case RobotMode::STOP:
      break;

    case RobotMode::CRAB_WALK:
      crab_walk();
      break;

    case RobotMode::WAVE_WALK:
      wave_gait();
      break;
    
    case  RobotMode::ROTATE:
      rotate();
      break;
  }
}
