/**
 * @file robot_controller.hpp
 * @brief Multi-Network Robot Controller for Phase 2 Architecture
 * 
 * This header defines the RobotController class for coordinating multiple
 * EtherCAT networks, each with their own motor chains. Enables complex
 * robotic systems with separate networks per limb/subsystem.
 */

#pragma once

#include "network_controller.hpp"
#include "motor_controller.hpp"
#include <string>
#include <map>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>
#include <future>
#include <chrono>
#include <mutex>

namespace synapticon_motor {

/**
 * @brief Network configuration information
 */
struct NetworkConfig {
    std::string network_name;           ///< Unique network identifier
    std::string interface_name;         ///< EtherCAT interface (e.g., "eth0")
    double update_frequency_hz;         ///< Network update frequency in Hz
    std::string description;            ///< Human-readable description
    int priority;                       ///< Network priority (0=highest)
    bool enabled;                       ///< Whether network is enabled
    
    // Default constructor for std::map compatibility
    NetworkConfig() 
        : network_name(""), interface_name(""), update_frequency_hz(250.0),
          description(""), priority(10), enabled(true) {}
    
    NetworkConfig(const std::string& name, const std::string& interface, 
                 double frequency = 250.0, const std::string& desc = "", 
                 int prio = 10, bool enable = true)
        : network_name(name), interface_name(interface), update_frequency_hz(frequency),
          description(desc), priority(prio), enabled(enable) {}
};

/**
 * @brief Robot-wide status information
 */
struct RobotStatus {
    bool initialized;                   ///< All networks initialized
    bool operational;                   ///< All networks operational
    int total_networks;                 ///< Number of configured networks
    int active_networks;                ///< Number of active networks
    int total_motors;                   ///< Total motors across all networks
    double global_cycle_time_us;        ///< Global cycle time in microseconds
    uint64_t global_cycle_count;        ///< Total synchronized cycles
    std::string last_error;             ///< Last error message
    bool emergency_stop_active;         ///< Emergency stop state
};

/**
 * @brief Multi-Network Robot Controller
 * 
 * Coordinates multiple EtherCAT networks for complex robotic systems.
 * Each network operates independently with its own thread while maintaining
 * synchronized timing for coordinated robot control.
 * 
 * Typical use cases:
 * - Humanoid robots (separate networks per limb)
 * - Multi-arm industrial systems
 * - Distributed motor control systems
 */
class RobotController {
public:
    /**
     * @brief Constructor
     * @param global_frequency_hz Global coordination frequency (default: 250Hz)
     */
    explicit RobotController(double global_frequency_hz = 250.0);
    
    /**
     * @brief Destructor - automatically shuts down all networks
     */
    ~RobotController();
    
    // Disable copy constructor and assignment (robot resources are unique)
    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;
    
    // === Network Management ===
    
    /**
     * @brief Add a new EtherCAT network to the robot
     * @param config Network configuration
     * @return true if network added successfully
     */
    bool addNetwork(const NetworkConfig& config);
    
    /**
     * @brief Remove a network from the robot
     * @param network_name Name of network to remove
     * @return true if network removed successfully
     */
    bool removeNetwork(const std::string& network_name);
    
    /**
     * @brief Get network controller by name
     * @param network_name Name of the network
     * @return Pointer to NetworkController or nullptr if not found
     */
    NetworkController* getNetwork(const std::string& network_name);
    
    /**
     * @brief Get all configured networks
     * @return Vector of network names
     */
    std::vector<std::string> getNetworkNames() const;
    
    /**
     * @brief Get network count
     * @return Number of configured networks
     */
    size_t getNetworkCount() const;
    
    // === Robot-Level Operations ===
    
    /**
     * @brief Initialize all networks
     * @return true if all networks initialized successfully
     */
    bool initializeAllNetworks();
    
    /**
     * @brief Start operation on all networks
     * @return true if all networks started successfully
     */
    bool startAllNetworks();
    
    /**
     * @brief Update all networks in synchronized fashion
     * 
     * Performs one coordinated update cycle across all networks.
     * Uses parallel execution with synchronization barriers.
     */
    void updateAllNetworks();
    
    /**
     * @brief Emergency stop all networks immediately
     */
    void emergencyStopAll();
    
    /**
     * @brief Shutdown all networks safely
     */
    void shutdownAll();
    
    // === Motor Access Across Networks ===
    
    /**
     * @brief Get motor controller from specific network
     * @param network_name Name of the network
     * @param slave_index EtherCAT slave index within that network
     * @return Pointer to MotorController or nullptr if not found
     */
    MotorController* getMotor(const std::string& network_name, int slave_index);
    
    /**
     * @brief Get all motor controllers across all networks
     * @return Vector of MotorController pointers
     */
    std::vector<MotorController*> getAllMotors();
    
    /**
     * @brief Get motors from specific network
     * @param network_name Name of the network
     * @return Vector of MotorController pointers from that network
     */
    std::vector<MotorController*> getNetworkMotors(const std::string& network_name);
    
    // === Coordinated Control ===
    
    /**
     * @brief Set positions for multiple networks simultaneously
     * @param network_positions Map of network_name -> vector of positions
     * @return true if all commands accepted
     */
    bool setNetworkPositions(const std::map<std::string, std::vector<double>>& network_positions);
    
    /**
     * @brief Set velocities for multiple networks simultaneously
     * @param network_velocities Map of network_name -> vector of velocities (RPM)
     * @return true if all commands accepted
     */
    bool setNetworkVelocities(const std::map<std::string, std::vector<double>>& network_velocities);
    
    /**
     * @brief Get feedback from all networks
     * @return Map of network_name -> vector of positions
     */
    std::map<std::string, std::vector<double>> getAllNetworkFeedback();
    
    // === Status and Diagnostics ===
    
    /**
     * @brief Check if all networks are operational
     * @return true if all networks are operational
     */
    bool allNetworksOperational() const;
    
    /**
     * @brief Get robot-wide status
     * @return RobotStatus structure
     */
    RobotStatus getRobotStatus() const;
    
    /**
     * @brief Get status of all networks
     * @return Map of network_name -> NetworkStatus
     */
    std::map<std::string, NetworkStatus> getAllNetworkStatus() const;
    
    /**
     * @brief Check if emergency stop is active
     * @return true if emergency stop is active
     */
    bool isEmergencyStopActive() const;
    
    // === Timing and Performance ===
    
    /**
     * @brief Get global update frequency
     * @return Global frequency in Hz
     */
    double getGlobalFrequency() const;
    
    /**
     * @brief Get global cycle time
     * @return Cycle time in microseconds
     */
    int getGlobalCycleTimeUs() const;
    
    /**
     * @brief Get total synchronized cycles completed
     * @return Number of cycles
     */
    uint64_t getGlobalCycleCount() const;

private:
    // === Core Data Structures ===
    std::map<std::string, std::unique_ptr<NetworkController>> networks_;  ///< All network controllers
    std::map<std::string, NetworkConfig> network_configs_;               ///< Network configurations
    
    // === Threading and Synchronization ===
    std::atomic<bool> initialized_;                 ///< Robot initialization state
    std::atomic<bool> operational_;                 ///< Robot operational state
    std::atomic<bool> emergency_stop_;              ///< Emergency stop flag
    std::atomic<bool> shutdown_requested_;          ///< Shutdown request flag
    
    // === Timing Control ===
    double global_frequency_hz_;                    ///< Global coordination frequency
    int global_cycle_time_us_;                     ///< Global cycle time in microseconds
    std::atomic<uint64_t> global_cycle_count_;     ///< Total synchronized cycles
    std::chrono::high_resolution_clock::time_point last_update_time_;  ///< Last update timestamp
    
    // === Thread Safety ===
    mutable std::mutex networks_mutex_;             ///< Thread safety for network operations
    mutable std::mutex status_mutex_;               ///< Thread safety for status updates
    std::string last_error_;                        ///< Last error message
    
    // === Internal Helper Methods ===
    
    /**
     * @brief Validate network configuration
     * @param config Network configuration to validate
     * @return true if configuration is valid
     */
    bool validateNetworkConfig(const NetworkConfig& config) const;
    
    /**
     * @brief Check if network name is unique
     * @param network_name Name to check
     * @return true if name is unique
     */
    bool isNetworkNameUnique(const std::string& network_name) const;
    
    /**
     * @brief Set error message (thread-safe)
     * @param error_message Error description
     */
    void setError(const std::string& error_message);
    
    /**
     * @brief Clear error state
     */
    void clearError();
    
    /**
     * @brief Update network in parallel thread
     * @param network_name Name of network to update
     * @return true if update successful
     */
    bool updateNetworkParallel(const std::string& network_name);
    
    /**
     * @brief Calculate timing statistics
     */
    void updateTimingStatistics();
};

} // namespace synapticon_motor