#include <Odometry.h>
#define ROBOT_MODE (ExecMode::GAME_ON)
Kinematics kinematics;
MotionControl motion(&kinematics);
Odometry odometry(&motion, &kinematics, ODOMETRY_TSAMPLE, GYRO_TSAMPLE); // Odm at 5ms, Gyro as 5ms
int main() {
      // Init odometry after debugger for avoiding calibration beep
  odometry.init();
    // Process Gyroscope (units in rad/s)
    odometry.processGyro();
    //Read the sensor
    odometry.getAllGyroRead();
    //Calibrate the noise (offset)
    odometry.calibrateGyroOffset();
    //Get the results
    odometry.update(); 
}