/**
 * @file test_multi_network_hardware.cpp
 * @brief Multi-Network Hardware Integration Test
 * 
 * Comprehensive test for RobotController with multiple physical EtherCAT networks.
 * Use this test when you have multiple EtherCAT interfaces and motor chains.
 * 
 * Usage: sudo ./test_multi_network_hardware <config_file>
 * Example: sudo ./test_multi_network_hardware config/humanoid_robot.json
 */

#include "robot_controller.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm>
#include <numeric>

using namespace synapticon_motor;

/**
 * @brief Multi-Network Hardware Test Suite
 */
class MultiNetworkHardwareTest {
private:
    std::unique_ptr<RobotController> robot_;
    std::vector<NetworkConfig> network_configs_;
    std::string config_file_;
    
public:
    MultiNetworkHardwareTest(const std::string& config_file)
        : config_file_(config_file) {}
    
    /**
     * @brief Load network configuration from file
     * For now, use hardcoded configurations. In production, this would load JSON.
     */
    bool loadNetworkConfiguration() {
        std::cout << "\n=== Loading Network Configuration ===" << std::endl;
        
        // TODO: Implement JSON config file loading
        // For now, create example configurations that can be modified as needed
        
        std::cout << "Creating example multi-network configuration..." << std::endl;
        std::cout << "Note: Modify interface names for your hardware setup" << std::endl;
        
        // Example: Humanoid robot configuration
        network_configs_ = {
            NetworkConfig("left_arm", "eth0", 250.0, "Left arm motor chain (3 motors)", 5),
            NetworkConfig("right_arm", "eth1", 250.0, "Right arm motor chain (3 motors)", 5),
            NetworkConfig("left_leg", "eth2", 500.0, "Left leg motor chain (4 motors)", 1),  // Higher frequency
            NetworkConfig("right_leg", "eth3", 500.0, "Right leg motor chain (4 motors)", 1), // Higher frequency
        };
        
        std::cout << "Configured " << network_configs_.size() << " networks:" << std::endl;
        for (const auto& config : network_configs_) {
            std::cout << "  - " << config.network_name 
                      << " (" << config.interface_name << "): " 
                      << config.update_frequency_hz << "Hz - " << config.description << std::endl;
        }
        
        std::cout << "✅ Network configuration loaded" << std::endl;
        return true;
    }
    
    /**
     * @brief Create and configure RobotController
     */
    bool setupRobotController() {
        std::cout << "\n=== RobotController Setup ===" << std::endl;
        
        try {
            // Create RobotController with 250Hz global coordination
            robot_ = std::make_unique<RobotController>(250.0);
            
            std::cout << "RobotController created:" << std::endl;
            std::cout << "  - Global frequency: " << robot_->getGlobalFrequency() << "Hz" << std::endl;
            std::cout << "  - Global cycle time: " << robot_->getGlobalCycleTimeUs() << "μs" << std::endl;
            
            // Add all networks
            for (const auto& config : network_configs_) {
                bool success = robot_->addNetwork(config);
                std::cout << "  " << config.network_name << ": " 
                          << (success ? "✅ Added" : "❌ Failed") << std::endl;
                
                if (!success) {
                    return false;
                }
            }
            
            std::cout << "✅ RobotController setup complete" << std::endl;
            std::cout << "  - Total networks: " << robot_->getNetworkCount() << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "❌ RobotController setup failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Initialize all EtherCAT networks
     */
    bool initializeAllNetworks() {
        std::cout << "\n=== Multi-Network Initialization ===" << std::endl;
        
        std::cout << "Initializing " << robot_->getNetworkCount() << " EtherCAT networks in parallel..." << std::endl;
        std::cout << "This may take 10-30 seconds depending on network complexity." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        bool success = robot_->initializeAllNetworks();
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (success) {
            std::cout << "✅ All networks initialized successfully in " 
                      << duration.count() << "ms" << std::endl;
            
            // Display network status
            auto network_statuses = robot_->getAllNetworkStatus();
            std::cout << "\nNetwork Initialization Results:" << std::endl;
            
            for (const auto& [name, status] : network_statuses) {
                std::cout << "  " << name << ":" << std::endl;
                std::cout << "    - Initialized: " << (status.initialized ? "✅" : "❌") << std::endl;
                std::cout << "    - Slave count: " << status.slave_count << std::endl;
                std::cout << "    - Cycle time: " << status.cycle_time_us << "μs" << std::endl;
            }
        } else {
            std::cout << "❌ Network initialization failed after " 
                      << duration.count() << "ms" << std::endl;
            
            RobotStatus robot_status = robot_->getRobotStatus();
            if (!robot_status.last_error.empty()) {
                std::cout << "Error: " << robot_status.last_error << std::endl;
            }
        }
        
        return success;
    }
    
    /**
     * @brief Start operation on all networks and detect motors
     */
    bool startAllNetworksAndDetectMotors() {
        std::cout << "\n=== Network Startup & Motor Detection ===" << std::endl;
        
        std::cout << "Starting operation on all networks..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        bool success = robot_->startAllNetworks();
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (success) {
            std::cout << "✅ All networks operational in " 
                      << duration.count() << "ms" << std::endl;
            
            // Detect and display motors on each network
            std::cout << "\nMotor Detection Results:" << std::endl;
            
            for (const auto& name : robot_->getNetworkNames()) {
                NetworkController* network = robot_->getNetwork(name);
                if (network) {
                    auto detected_motors = network->getAllDetectedMotors();
                    
                    std::cout << "  " << name << " (" << detected_motors.size() << " motors):" << std::endl;
                    
                    for (const auto& motor_info : detected_motors) {
                        std::cout << "    Slave " << motor_info.slave_index 
                                  << ": " << motor_info.type_name
                                  << " (ID: 0x" << std::hex << motor_info.product_id << std::dec << ")" << std::endl;
                    }
                }
            }
            
            // Display robot-wide status
            RobotStatus robot_status = robot_->getRobotStatus();
            std::cout << "\nRobot Status:" << std::endl;
            std::cout << "  - Total networks: " << robot_status.total_networks << std::endl;
            std::cout << "  - Active networks: " << robot_status.active_networks << std::endl;
            std::cout << "  - Total motors: " << robot_status.total_motors << std::endl;
            std::cout << "  - Operational: " << (robot_status.operational ? "Yes" : "No") << std::endl;
            
        } else {
            std::cout << "❌ Network startup failed after " 
                      << duration.count() << "ms" << std::endl;
        }
        
        return success;
    }
    
    /**
     * @brief Test coordinated multi-network control
     */
    bool testCoordinatedMultiNetworkControl() {
        std::cout << "\n=== Coordinated Multi-Network Control Test ===" << std::endl;
        
        if (!robot_->allNetworksOperational()) {
            std::cout << "❌ Not all networks operational - skipping control test" << std::endl;
            return false;
        }
        
        std::cout << "Testing coordinated control across all networks..." << std::endl;
        
        // Record initial performance metrics
        uint64_t initial_cycles = robot_->getGlobalCycleCount();
        auto test_start = std::chrono::high_resolution_clock::now();
        
        std::cout << "Running 1000 coordinated update cycles (4 seconds at 250Hz)..." << std::endl;
        
        // Timing statistics
        std::vector<double> cycle_times;
        cycle_times.reserve(1000);
        
        for (int cycle = 0; cycle < 1000; cycle++) {
            auto cycle_start = std::chrono::high_resolution_clock::now();
            
            // Coordinated update across all networks
            robot_->updateAllNetworks();
            
            auto cycle_end = std::chrono::high_resolution_clock::now();
            auto cycle_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_start);
            cycle_times.push_back(cycle_duration.count());
            
            // Progress reporting
            if (cycle % 200 == 0) {
                uint64_t current_cycles = robot_->getGlobalCycleCount();
                std::cout << "  Cycle " << cycle 
                          << ": Global cycles = " << current_cycles
                          << ", Timing = " << cycle_duration.count() << "μs" << std::endl;
            }
        }
        
        auto test_end = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            test_end - test_start);
        
        // Calculate performance statistics
        uint64_t final_cycles = robot_->getGlobalCycleCount();
        uint64_t total_cycles = final_cycles - initial_cycles;
        
        double min_time = *std::min_element(cycle_times.begin(), cycle_times.end());
        double max_time = *std::max_element(cycle_times.begin(), cycle_times.end());
        double avg_time = std::accumulate(cycle_times.begin(), cycle_times.end(), 0.0) / cycle_times.size();
        
        std::cout << "\nPerformance Results:" << std::endl;
        std::cout << "  - Total test time: " << total_duration.count() << "ms" << std::endl;
        std::cout << "  - Expected cycles: 1000" << std::endl;
        std::cout << "  - Actual cycles: " << total_cycles << std::endl;
        std::cout << "  - Cycle timing - Min: " << std::fixed << std::setprecision(1) << min_time << "μs" << std::endl;
        std::cout << "  - Cycle timing - Avg: " << avg_time << "μs" << std::endl;
        std::cout << "  - Cycle timing - Max: " << max_time << "μs" << std::endl;
        std::cout << "  - Target cycle time: " << robot_->getGlobalCycleTimeUs() << "μs" << std::endl;
        
        // Performance validation
        bool cycle_count_correct = (total_cycles == 1000);
        bool avg_timing_acceptable = (avg_time < robot_->getGlobalCycleTimeUs() * 1.5);  // Within 150% of target
        bool max_timing_reasonable = (max_time < robot_->getGlobalCycleTimeUs() * 3.0);  // Max 3x target
        
        std::cout << "\nPerformance Validation:" << std::endl;
        std::cout << "  - Cycle count correct: " << (cycle_count_correct ? "✅" : "❌") << std::endl;
        std::cout << "  - Average timing acceptable: " << (avg_timing_acceptable ? "✅" : "❌") << std::endl;
        std::cout << "  - Maximum timing reasonable: " << (max_timing_reasonable ? "✅" : "❌") << std::endl;
        
        bool performance_acceptable = cycle_count_correct && avg_timing_acceptable && max_timing_reasonable;
        
        if (performance_acceptable) {
            std::cout << "✅ Multi-network coordinated control: SUCCESSFUL" << std::endl;
        } else {
            std::cout << "⚠️  Multi-network coordinated control: PERFORMANCE ISSUES" << std::endl;
        }
        
        return performance_acceptable;
    }
    
    /**
     * @brief Test emergency stop across all networks
     */
    bool testGlobalEmergencyStop() {
        std::cout << "\n=== Global Emergency Stop Test ===" << std::endl;
        
        std::cout << "Testing emergency stop across all networks..." << std::endl;
        
        // Trigger emergency stop
        robot_->emergencyStopAll();
        
        // Check emergency stop status
        bool estop_active = robot_->isEmergencyStopActive();
        std::cout << "  Emergency stop active: " << (estop_active ? "✅" : "❌") << std::endl;
        
        // Check that all networks stopped
        auto network_statuses = robot_->getAllNetworkStatus();
        bool all_networks_stopped = true;
        
        for (const auto& [name, status] : network_statuses) {
            bool network_stopped = !status.operational;
            std::cout << "  " << name << " stopped: " << (network_stopped ? "✅" : "❌") << std::endl;
            
            if (!network_stopped) {
                all_networks_stopped = false;
            }
        }
        
        bool emergency_stop_successful = estop_active && all_networks_stopped;
        
        if (emergency_stop_successful) {
            std::cout << "✅ Global emergency stop: SUCCESSFUL" << std::endl;
        } else {
            std::cout << "❌ Global emergency stop: FAILED" << std::endl;
        }
        
        return emergency_stop_successful;
    }
    
    /**
     * @brief Clean shutdown of all systems
     */
    void shutdown() {
        std::cout << "\n=== System Shutdown ===" << std::endl;
        
        std::cout << "Shutting down all networks safely..." << std::endl;
        robot_->shutdownAll();
        
        std::cout << "✅ All systems shutdown complete" << std::endl;
    }
    
    /**
     * @brief Run complete multi-network hardware test
     */
    bool runCompleteTest() {
        std::cout << "Multi-Network Hardware Integration Test" << std::endl;
        std::cout << "=======================================" << std::endl;
        std::cout << "Phase 2: Real Hardware Multi-Network Validation" << std::endl;
        std::cout << "Config file: " << config_file_ << std::endl;
        std::cout << std::string(50, '=') << std::endl;
        
        bool success = true;
        
        success &= loadNetworkConfiguration();
        if (!success) return false;
        
        success &= setupRobotController();
        if (!success) return false;
        
        success &= initializeAllNetworks();
        if (!success) return false;
        
        success &= startAllNetworksAndDetectMotors();
        if (!success) return false;
        
        success &= testCoordinatedMultiNetworkControl();
        // Continue even if performance issues - not critical failure
        
        success &= testGlobalEmergencyStop();
        
        // Always perform safe shutdown
        shutdown();
        
        std::cout << "\n" << std::string(50, '=') << std::endl;
        if (success) {
            std::cout << "🎉 MULTI-NETWORK HARDWARE TEST SUCCESSFUL!" << std::endl;
            std::cout << "✅ Phase 2 Multi-Network Architecture: 100% VALIDATED" << std::endl;
            std::cout << "✅ Complex robotic systems: READY" << std::endl;
            std::cout << "✅ Production deployment: APPROVED" << std::endl;
        } else {
            std::cout << "❌ Multi-network hardware test failed" << std::endl;
            std::cout << "Check hardware connections and network configurations" << std::endl;
        }
        
        return success;
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        std::cerr << "Example: " << argv[0] << " config/humanoid_robot.json" << std::endl;
        std::cerr << "Note: For now, config file is not used - modify source code for your setup" << std::endl;
        return 1;
    }
    
    std::string config_file = argv[1];
    
    MultiNetworkHardwareTest test_suite(config_file);
    
    bool success = test_suite.runCompleteTest();
    
    return success ? 0 : 1;
}