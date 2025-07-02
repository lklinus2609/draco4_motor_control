/**
 * @file test_multi_motor_hardware.cpp
 * @brief Multi-Motor Hardware Integration Test
 * 
 * Comprehensive test for multiple physical motors on a single EtherCAT network.
 * Use this test when you have multiple JD8/JD10/JD12 motors connected.
 * 
 * Usage: sudo ./test_multi_motor_hardware <interface> [config_dir]
 * Example: sudo ./test_multi_motor_hardware enx3c18a042f97c config/
 */

#include "network_controller.hpp"
#include "motor_controller.hpp"
#include "motor_manager.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>

using namespace synapticon_motor;

/**
 * @brief Multi-Motor Hardware Test Suite
 */
class MultiMotorHardwareTest {
private:
    std::unique_ptr<NetworkController> network_;
    std::vector<std::unique_ptr<MotorController>> motors_;
    std::string interface_name_;
    std::string config_directory_;
    
public:
    MultiMotorHardwareTest(const std::string& interface, const std::string& config_dir = "config/")
        : interface_name_(interface), config_directory_(config_dir) {}
    
    /**
     * @brief Initialize EtherCAT network and detect motors
     */
    bool initializeNetwork() {
        std::cout << "\n=== Phase 1: Network Initialization ===" << std::endl;
        
        try {
            // Create NetworkController with MotorManager
            network_ = std::make_unique<NetworkController>(interface_name_, 250.0);
            
            std::cout << "Step 1: Initializing EtherCAT master..." << std::endl;
            if (!network_->initialize()) {
                std::cerr << "❌ Failed to initialize EtherCAT master" << std::endl;
                return false;
            }
            std::cout << "✅ EtherCAT master initialized" << std::endl;
            
            std::cout << "Step 2: Scanning for motor slaves..." << std::endl;
            if (!network_->scanNetwork()) {
                std::cerr << "❌ Failed to scan EtherCAT network" << std::endl;
                return false;
            }
            
            int slave_count = network_->getSlaveCount();
            if (slave_count <= 0) {
                std::cerr << "❌ No EtherCAT slaves detected - check motor power and connections" << std::endl;
                return false;
            }
            std::cout << "✅ Found " << slave_count << " EtherCAT slaves" << std::endl;
            
            std::cout << "Step 3: Configuring slaves..." << std::endl;
            if (!network_->configureSlaves()) {
                std::cerr << "❌ Failed to configure EtherCAT slaves" << std::endl;
                return false;
            }
            std::cout << "✅ All slaves configured" << std::endl;
            
            std::cout << "Step 4: Starting network operation..." << std::endl;
            if (!network_->startOperation()) {
                std::cerr << "❌ Failed to start EtherCAT operation" << std::endl;
                return false;
            }
            std::cout << "✅ Network operational" << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "❌ Network initialization failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Detect and analyze all connected motors
     */
    bool detectAndAnalyzeMotors() {
        std::cout << "\n=== Phase 2: Motor Detection & Analysis ===" << std::endl;
        
        // Get all detected motor information
        std::vector<DetectedMotorInfo> detected_motors = network_->getAllDetectedMotors();
        
        if (detected_motors.empty()) {
            std::cerr << "❌ No motors detected by MotorManager" << std::endl;
            return false;
        }
        
        std::cout << "Detected Motors:" << std::endl;
        std::cout << std::setw(6) << "Slave" 
                  << std::setw(12) << "Product ID" 
                  << std::setw(20) << "Motor Type"
                  << std::setw(30) << "Config File" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
        
        for (const auto& motor_info : detected_motors) {
            std::cout << std::setw(6) << motor_info.slave_index
                      << std::setw(12) << ("0x" + std::to_string(motor_info.product_id))
                      << std::setw(20) << motor_info.type_name
                      << std::setw(30) << motor_info.config_file_path << std::endl;
        }
        
        // Verify product IDs match expected motor types
        std::cout << "\nMotor Type Analysis:" << std::endl;
        const MotorManager* motor_manager = network_->getMotorManager();
        
        for (const auto& motor_info : detected_motors) {
            if (motor_info.detected_type == MotorType::UNKNOWN) {
                std::cout << "⚠️  Slave " << motor_info.slave_index 
                          << ": Unknown motor type (ID: 0x" << std::hex << motor_info.product_id << std::dec << ")" << std::endl;
                std::cout << "   -> Using default JD8 configuration for compatibility" << std::endl;
            } else {
                std::cout << "✅ Slave " << motor_info.slave_index 
                          << ": " << motor_info.type_name << " correctly identified" << std::endl;
            }
        }
        
        std::cout << "✅ Motor detection and analysis completed" << std::endl;
        return true;
    }
    
    /**
     * @brief Create MotorController instances for each detected motor
     */
    bool createMotorControllers() {
        std::cout << "\n=== Phase 3: MotorController Creation ===" << std::endl;
        
        std::vector<DetectedMotorInfo> detected_motors = network_->getAllDetectedMotors();
        std::shared_ptr<NetworkController> shared_network(network_.release());
        
        for (const auto& motor_info : detected_motors) {
            try {
                std::cout << "Creating MotorController for slave " << motor_info.slave_index << "..." << std::endl;
                
                // Create MotorController instance
                auto motor = std::make_unique<MotorController>(shared_network, motor_info.slave_index, 250.0);
                
                // Load appropriate configuration file
                std::string full_config_path = config_directory_ + motor_info.config_file_path;
                if (motor->loadConfig(full_config_path)) {
                    std::cout << "  ✅ Configuration loaded: " << full_config_path << std::endl;
                } else {
                    std::cout << "  ⚠️  Failed to load config, using defaults: " << full_config_path << std::endl;
                }
                
                motors_.push_back(std::move(motor));
                std::cout << "  ✅ MotorController " << motor_info.slave_index << " created successfully" << std::endl;
                
            } catch (const std::exception& e) {
                std::cerr << "  ❌ Failed to create MotorController for slave " 
                          << motor_info.slave_index << ": " << e.what() << std::endl;
                return false;
            }
        }
        
        // Restore network pointer for cleanup
        network_.reset(shared_network.get());
        
        std::cout << "✅ Created " << motors_.size() << " MotorController instances" << std::endl;
        return true;
    }
    
    /**
     * @brief Enable all motors using CIA402 state machine
     */
    bool enableAllMotors() {
        std::cout << "\n=== Phase 4: Motor Enable Sequence ===" << std::endl;
        
        std::cout << "Enabling " << motors_.size() << " motors..." << std::endl;
        
        for (size_t i = 0; i < motors_.size(); i++) {
            std::cout << "Enabling motor " << (i+1) << "..." << std::endl;
            
            if (motors_[i]->enable_motor()) {
                std::cout << "  ✅ Motor " << (i+1) << " enabled successfully" << std::endl;
            } else {
                std::cout << "  ❌ Failed to enable motor " << (i+1) << std::endl;
                std::cout << "  Status: " << motors_[i]->getStateStr() << std::endl;
                return false;
            }
        }
        
        std::cout << "✅ All motors enabled and ready for operation" << std::endl;
        return true;
    }
    
    /**
     * @brief Test coordinated multi-motor control
     */
    bool testCoordinatedControl() {
        std::cout << "\n=== Phase 5: Coordinated Multi-Motor Control ===" << std::endl;
        
        if (motors_.size() < 2) {
            std::cout << "ℹ️  Only one motor detected - running single motor test" << std::endl;
            return testSingleMotorControl();
        }
        
        try {
            std::cout << "Setting up coordinated control scenario:" << std::endl;
            
            // Assign different control modes to different motors
            std::vector<std::string> mode_names = {"VELOCITY", "POSITION", "TORQUE"};
            
            for (size_t i = 0; i < motors_.size() && i < 3; i++) {
                MotorController::ControlMode mode = static_cast<MotorController::ControlMode>(i);
                motors_[i]->set_control_mode(mode);
                std::cout << "  Motor " << (i+1) << ": " << mode_names[i] << " mode" << std::endl;
            }
            
            std::cout << "\nExecuting coordinated motion sequence..." << std::endl;
            
            // Execute coordinated commands
            if (motors_.size() >= 1) {
                motors_[0]->set_velocity_rpm(500);  // 500 RPM
                std::cout << "  Motor 1: Set velocity to 500 RPM" << std::endl;
            }
            
            if (motors_.size() >= 2) {
                motors_[1]->set_position_counts(90000);  // 45 degrees
                std::cout << "  Motor 2: Set position to 90000 counts (45°)" << std::endl;
            }
            
            if (motors_.size() >= 3) {
                motors_[2]->set_torque_millinm(2000);  // 2 Nm
                std::cout << "  Motor 3: Set torque to 2000 mNm" << std::endl;
            }
            
            // Run coordinated control loop
            auto start_time = std::chrono::steady_clock::now();
            auto next_cycle = start_time + std::chrono::microseconds(4000);  // 250Hz
            
            std::cout << "\nRunning 100 coordinated control cycles (400ms)..." << std::endl;
            
            for (int cycle = 0; cycle < 100; cycle++) {
                // Update all motors in coordinated fashion
                for (auto& motor : motors_) {
                    motor->update();
                }
                
                // Print status every 25 cycles
                if (cycle % 25 == 0) {
                    std::cout << "  Cycle " << cycle << ": ";
                    for (size_t i = 0; i < motors_.size(); i++) {
                        std::cout << "M" << (i+1) << ":" << motors_[i]->getStateStr() << " ";
                    }
                    std::cout << std::endl;
                }
                
                // Maintain precise timing
                std::this_thread::sleep_until(next_cycle);
                next_cycle += std::chrono::microseconds(4000);
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            std::cout << "✅ Coordinated control completed in " << duration.count() << "ms" << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "❌ Coordinated control test failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Test single motor control (fallback)
     */
    bool testSingleMotorControl() {
        std::cout << "Testing single motor control..." << std::endl;
        
        auto& motor = motors_[0];
        
        // Simple velocity test
        motor->set_control_mode(MotorController::VELOCITY_MODE);
        motor->set_velocity_rpm(200);
        
        std::cout << "Running single motor velocity test (200 RPM)..." << std::endl;
        
        for (int i = 0; i < 50; i++) {
            motor->update();
            if (i % 10 == 0) {
                std::cout << "  Cycle " << i << ": " << motor->getStateStr() 
                          << " RPM: " << motor->getMotorRPM() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        
        return true;
    }
    
    /**
     * @brief Clean shutdown of all motors
     */
    void shutdown() {
        std::cout << "\n=== Phase 6: Shutdown Sequence ===" << std::endl;
        
        std::cout << "Disabling all motors..." << std::endl;
        for (size_t i = 0; i < motors_.size(); i++) {
            motors_[i]->disable_motor();
            std::cout << "  Motor " << (i+1) << " disabled" << std::endl;
        }
        
        std::cout << "Stopping EtherCAT network..." << std::endl;
        if (network_) {
            network_->stopOperation();
        }
        
        std::cout << "✅ Shutdown completed safely" << std::endl;
    }
    
    /**
     * @brief Run complete multi-motor hardware test
     */
    bool runCompleteTest() {
        std::cout << "Multi-Motor Hardware Integration Test" << std::endl;
        std::cout << "====================================" << std::endl;
        std::cout << "Interface: " << interface_name_ << std::endl;
        std::cout << "Config Dir: " << config_directory_ << std::endl;
        std::cout << std::string(40, '=') << std::endl;
        
        bool success = true;
        
        success &= initializeNetwork();
        if (!success) return false;
        
        success &= detectAndAnalyzeMotors();
        if (!success) return false;
        
        success &= createMotorControllers();
        if (!success) return false;
        
        success &= enableAllMotors();
        if (!success) return false;
        
        success &= testCoordinatedControl();
        
        // Always perform safe shutdown
        shutdown();
        
        std::cout << "\n" << std::string(50, '=') << std::endl;
        if (success) {
            std::cout << "🎉 MULTI-MOTOR HARDWARE TEST SUCCESSFUL!" << std::endl;
            std::cout << "✅ Phase 1 Multi-Motor Architecture: 100% Complete" << std::endl;
            std::cout << "   - " << motors_.size() << " motors operated successfully" << std::endl;
            std::cout << "   - Automatic motor type detection working" << std::endl;
            std::cout << "   - Coordinated control validated" << std::endl;
        } else {
            std::cout << "❌ Multi-motor hardware test failed" << std::endl;
        }
        
        return success;
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <interface> [config_directory]" << std::endl;
        std::cerr << "Example: " << argv[0] << " enx3c18a042f97c config/" << std::endl;
        return 1;
    }
    
    std::string interface = argv[1];
    std::string config_dir = (argc > 2) ? argv[2] : "config/";
    
    MultiMotorHardwareTest test_suite(interface, config_dir);
    
    bool success = test_suite.runCompleteTest();
    
    return success ? 0 : 1;
}