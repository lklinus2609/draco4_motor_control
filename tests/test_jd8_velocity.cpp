/**
 * @file test_jd8_velocity.cpp
 * @brief JD8 velocity scaling test
 */

#include "motor_controller.hpp"
#include "network_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <iomanip>
#include <memory>

volatile bool running = true;
void signal_handler(int) { running = false; }

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    std::string interface_name = argv[1];
    
    std::cout << "=== JD8 VELOCITY TEST ===" << std::endl;
    std::cout << "Testing corrected velocity scaling" << std::endl;
    
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
    
    std::cout << "Motor initialized successfully" << std::endl;
    
    // Load and upload new configuration
    std::cout << "\nLoading new motor configuration..." << std::endl;
    if (!motor.loadConfig("../config/JDLINK8_config_file.csv")) {
        std::cerr << "Failed to load configuration file" << std::endl;
        return 1;
    }
    
    std::cout << "Uploading configuration to motor..." << std::endl;
    if (!motor.uploadConfig()) {
        std::cerr << "Failed to upload configuration to motor" << std::endl;
        return 1;
    }
    
    std::cout << "New configuration uploaded successfully" << std::endl;
    
    // Test 157.5 RPM (rounded to 158 for integer input)
    std::cout << "\nTesting 158 RPM (target was 157.5)..." << std::endl;
    int32_t start_pos = motor.getPosition();
    motor.set_velocity_rpm(158);
    
    // Start timing measurement
    auto start_time = std::chrono::steady_clock::now();
    
    double total_update_time = 0.0;
    
    // Precise timing - sync with EtherCAT cycles using configured frequency
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(motor.getCycleTimeUs());
    
    // Calculate cycles for 10 second test duration
    int total_cycles = static_cast<int>(10.0 * motor.getUpdateFrequencyHz());
    int log_interval = static_cast<int>(motor.getUpdateFrequencyHz()); // Log every 1 second
    
    for (int i = 0; i < total_cycles && running; i++) {
        auto update_start = std::chrono::steady_clock::now();
        network->performCommunicationCycle();  // Handle EtherCAT communication
        motor.update();                        // Handle motor control logic
        auto update_end = std::chrono::steady_clock::now();
        
        total_update_time += std::chrono::duration_cast<std::chrono::microseconds>(update_end - update_start).count() / 1000.0;
        
        if (i % log_interval == 0) { // Log every second
            int32_t current_pos = motor.getPosition();
            double current_vel = motor.getMotorRPM();
            double elapsed_time_ms = i * motor.getCycleTimeMs();
            std::cout << "t=" << static_cast<int>(elapsed_time_ms) << "ms: " << current_vel << " RPM, pos=" 
                      << current_pos << " (change: " << (current_pos - start_pos) << ")" << std::endl;
        }
        
        // Sleep until next cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(motor.getCycleTimeUs());
    }
    
    // End timing measurement
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    int32_t end_pos = motor.getPosition();
    int32_t position_change = end_pos - start_pos;
    double actual_rpm = synapticon_motor::MotorConstants::output_rpm_from_position_change(position_change, elapsed_ms / 1000.0, motor.getConfig());
    
    std::cout << "Results:" << std::endl;
    std::cout << "  Expected duration: 10000ms" << std::endl;
    std::cout << "  Actual duration: " << elapsed_ms << "ms" << std::endl;
    std::cout << "  Timing accuracy: " << std::fixed << std::setprecision(1) 
              << (100.0 * elapsed_ms / 10000.0) << "% of expected" << std::endl;
    std::cout << "  motor.update() total time: " << std::fixed << std::setprecision(1) << total_update_time << "ms" << std::endl;
    std::cout << "  motor.update() avg per cycle: " << std::fixed << std::setprecision(2) << (total_update_time / total_cycles) << "ms" << std::endl;
    std::cout << "  sleep_until total time: " << std::fixed << std::setprecision(1) << (total_cycles * motor.getCycleTimeMs()) << "ms" << std::endl;
    std::cout << "  Overhead/other: " << std::fixed << std::setprecision(1) << (elapsed_ms - total_update_time - 10000.0) << "ms" << std::endl;
    std::cout << "  Position change: " << (end_pos - start_pos) << " counts" << std::endl;
    std::cout << "  Calculated RPM: " << std::fixed << std::setprecision(2) << actual_rpm << std::endl;
    std::cout << "  Velocity reading: " << std::fixed << std::setprecision(2) << motor.getMotorRPM() << " RPM" << std::endl;
    
    if (std::abs(actual_rpm - 157.5) < 10.0) {
        std::cout << "SUCCESS: Motor moving at correct speed" << std::endl;
    }
    
    motor.set_velocity_rpm(0);
    motor.disable_motor();
    network->stopOperation();
    
    return 0;
}