#include "JAKAZuRobot.h"
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

JAKAZuRobot robot;

int main(void)
{
    
    int sensor_compensation, compliance_type;
    setlocale(LC_ALL, "");
    // robot.login_in(argv[1]);
    string default_ip = "10.5.5.100";
    robot.login_in(default_ip.c_str());
    robot.set_torque_sensor_mode(1);  
    robot.disable_force_control();
    robot.set_compliant_type(1, 0);  
    robot.get_compliant_type(&sensor_compensation, &compliance_type);  
    std::cout << "sensor_compensation: " << sensor_compensation << std::endl;
    std::cout << "compliance_type: " << compliance_type << std::endl;



    return 0;
}
