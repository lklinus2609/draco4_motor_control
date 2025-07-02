#include "motor_controller.hpp"
#include "network_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <memory>

volatile bool running = true;
void signal_handler(int) { running = false; }

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        std::cerr << "Example: " << argv[0] << " enx3c18a042f97c" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    std::string interface_name = argv[1];
    
    std::cout << "=== JD8 Torque Control Test ===" << std::endl;
    std::cout << "Interface: " << interface_name << std::endl;
    std::cout << "Rated torque: 6 Nm, Testing at 50% limit (3 Nm)" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    // Create network controller and initialize EtherCAT
    auto network = std::make_shared<synapticon_motor::NetworkController>(interface_name, 250.0);
    
    if (!network->initialize() || !network->scanNetwork() || 
        !network->configureSlaves() || !network->startOperation()) {
        std::cerr << "Network initialization failed" << std::endl;
        return 1;
    }
    
    // Create motor controller for slave 1
    synapticon_motor::MotorController motor(network, 1, 250.0);
    
    if (!motor.enable_motor()) {
        std::cerr << "Motor enable failed" << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Testing Torque Control ===" << std::endl;
    
    // Test 1: Small positive torque
    std::cout << "\n--- Test: Small torque (200 mNm) ---" << std::endl;
    motor.set_torque_millinm(200);
    
    for (int cycle = 0; cycle < 1000 && running; cycle++) {
        network->performCommunicationCycle();  // Handle EtherCAT communication
        motor.update();                        // Handle motor control logic
        
        if (cycle % 100 == 0) {
            std::cout << "Cycle " << cycle 
                      << " | State: " << motor.getStateStr()
                      << " | Actual: " << motor.getTorque() << " mNm"
                      << " | Position: " << motor.getPosition() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(motor.getCycleTimeUs()));
    }
    
    // Test 2: Safety limit test
    std::cout << "\n--- Test: Safety limit (5000→3000 mNm) ---" << std::endl;
    motor.set_torque_millinm(5000);  // Should be clamped
    
    for (int cycle = 0; cycle < 1000 && running; cycle++) {
        network->performCommunicationCycle();  // Handle EtherCAT communication
        motor.update();                        // Handle motor control logic
        
        if (cycle % 100 == 0) {
            std::cout << "Cycle " << cycle 
                      << " | Actual: " << motor.getTorque() << " mNm" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(motor.getCycleTimeUs()));
    }
    
    // Cleanup
    std::cout << "\n--- Shutting down ---" << std::endl;
    motor.set_torque_millinm(0);
    for (int i = 0; i < 100; i++) {
        network->performCommunicationCycle();
        motor.update();
        std::this_thread::sleep_for(std::chrono::microseconds(motor.getCycleTimeUs()));
    }
    
    motor.disable_motor();
    network->stopOperation();
    
    std::cout << "Torque control test completed!" << std::endl;
    return 0;
}
