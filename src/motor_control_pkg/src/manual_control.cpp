#include "SparkMax.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>


void setRawMode(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt); 
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); 
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
    }
}

int main() {
    try {
        SparkMax motor("can0", 1);   
        SparkMax actuator("can0", 3); 

        motor.SetIdleMode(IdleMode::kBrake);
        motor.SetMotorType(MotorType::kBrushless);
        motor.BurnFlash();

        actuator.SetIdleMode(IdleMode::kBrake);
        actuator.SetMotorType(MotorType::kBrushed);
        actuator.BurnFlash();

        std::cout << "Controls:" << std::endl;
        std::cout << "W: Motor forward" << std::endl;
        std::cout << "S: Motor backward" << std::endl;
        std::cout << "A: Extend actuator" << std::endl;
        std::cout << "D: Retract actuator" << std::endl;
        std::cout << "Q: Quit" << std::endl;

        setRawMode(true); 

        char input = '\0'; 
        while (input != 'q' && input != 'Q') { 
            motor.Heartbeat();
            actuator.Heartbeat();

            if (read(STDIN_FILENO, &input, 1) == 1) {
                switch (input) {
                    case 'w':
                    case 'W':
                        motor.SetDutyCycle(0.1); 
                        std::cout << "Moving motor forward..." << std::endl;
                        break;
                    case 's':
                    case 'S':
                        motor.SetDutyCycle(-0.1); 
                        std::cout << "Moving motor backward..." << std::endl;
                        break;
                    case 'a':
                    case 'A':
                        actuator.SetDutyCycle(0.25); 
                        std::cout << "Extending actuator..." << std::endl;
                        break;
                    case 'd':
                    case 'D':
                        actuator.SetDutyCycle(-0.25); 
                        std::cout << "Retracting actuator..." << std::endl;
                        break;
                    case 'q':
                    case 'Q':
                        std::cout << "Quitting..." << std::endl;
                        break;
                    default:
                        break; 
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
        }

        setRawMode(false); 

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        setRawMode(false); 
        return -1;
    }

    return 0;
}
