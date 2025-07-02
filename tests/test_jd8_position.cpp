#include "motor_controller.hpp"
#include "network_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    // Create network controller and initialize EtherCAT
    auto network = std::make_shared<synapticon_motor::NetworkController>(argv[1], 250.0);
    
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
    
    std::cout << "=== Position Control Test (90 Degree Output Shaft Move) ===" << std::endl;
    
    int32_t start_output_pos = motor.getOutputPos();
    int32_t start_motor_pos = motor.getPosition();
    std::cout << "Starting output shaft position: " << start_output_pos << std::endl;
    std::cout << "Starting motor shaft position: " << start_motor_pos << std::endl;
    
    // Move output shaft by 90 degrees (1/4 revolution)
    uint32_t counts_per_rev = synapticon_motor::MotorConstants::getCountsPerRev(motor.getConfig());
    double gear_ratio = synapticon_motor::MotorConstants::getGearReductionRatio(motor.getConfig());
    int32_t target_offset = counts_per_rev / 4;  // 90 degrees = 131072 counts
    int32_t target_pos = start_output_pos + target_offset;
    
    std::cout << "Target output shaft position: " << target_pos << " (change: +" << target_offset << " counts = 90 degrees)" << std::endl;
    std::cout << "Expected motor shaft movement: " << (target_offset * gear_ratio) << " counts" << std::endl;
    
    motor.set_position_counts(target_pos);
    
    // Precise timing using configured frequency
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(motor.getCycleTimeUs());
    
    for (int i = 0; i < 400; i++) {
        network->performCommunicationCycle();  // Handle EtherCAT communication
        motor.update();                        // Handle motor control logic
        
        if (i % 50 == 0) {
            int32_t motor_shaft_pos = motor.getPosition();
            int32_t output_shaft_pos = motor.getOutputPos();
            int32_t output_error = target_pos - output_shaft_pos;
            
            std::cout << "Cycle " << i 
                      << " | Motor: " << motor_shaft_pos 
                      << " | Output: " << output_shaft_pos
                      << " | Target: " << target_pos
                      << " | Error: " << output_error << " counts" << std::endl;
        }
        
        // Sleep until next cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(motor.getCycleTimeUs());
    }
    
    motor.disable_motor();
    network->stopOperation();
    
    std::cout << "\nPosition control test completed!" << std::endl;
    return 0;
}
