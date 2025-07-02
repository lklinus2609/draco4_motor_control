/**
 * @file network_controller.hpp
 * @brief EtherCAT Network Controller for Multi-Motor Management
 * 
 * This header defines the NetworkController class for managing EtherCAT networks
 * with multiple motor controllers. It extracts network-level operations from
 * MotorController to enable multi-motor and multi-network architectures.
 * 
 * @note Supports configurable frequencies and multiple slave management
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <chrono>
#include <mutex>

#include "motor_pdo_structures.hpp"
#include "motor_manager.hpp"

extern "C" {
#include <soem/ethercat.h>
}

namespace synapticon_motor {

// Forward declaration
class MotorController;

/**
 * @brief Information about an EtherCAT slave on the network
 */
struct SlaveInfo {
    int slave_index;               ///< EtherCAT slave index (1-based)
    std::string slave_name;        ///< Slave name from EtherCAT
    uint32_t product_id;           ///< EtherCAT product ID
    uint16_t vendor_id;            ///< EtherCAT vendor ID
    uint16_t current_state;        ///< Current EtherCAT state
    size_t output_pdo_size;        ///< Output PDO size in bytes
    size_t input_pdo_size;         ///< Input PDO size in bytes
    OutputPDO* output_pdo;         ///< Pointer to output PDO in IOmap
    InputPDO* input_pdo;           ///< Pointer to input PDO in IOmap
    bool configured;               ///< Whether slave has been configured
};

/**
 * @brief Network status information
 */
struct NetworkStatus {
    bool initialized;              ///< EtherCAT master initialized
    bool operational;              ///< Network in operational state
    int slave_count;               ///< Number of detected slaves
    int expected_wkc;              ///< Expected working counter
    int actual_wkc;                ///< Actual working counter from last cycle
    uint64_t cycle_count;          ///< Total communication cycles completed
    double cycle_time_us;          ///< Actual cycle time in microseconds
    std::string last_error;        ///< Last error message
};

/**
 * @brief EtherCAT Network Controller for Multi-Motor Management
 * 
 * Manages a single EtherCAT network with multiple motor slaves.
 * Provides centralized network communication and coordination.
 * Thread-safe for concurrent motor access to PDO data.
 */
class NetworkController {
public:
    /**
     * @brief Constructor
     * @param interface_name EtherCAT network interface (e.g., "enp2s0")
     * @param update_frequency_hz Network update frequency in Hz (default: 250.0)
     */
    explicit NetworkController(const std::string& interface_name, 
                              double update_frequency_hz = MotorConstants::DEFAULT_RT_FREQUENCY_HZ);
    
    /**
     * @brief Destructor - automatically stops network operation
     */
    ~NetworkController();
    
    // Delete copy constructor and assignment (network resources are unique)
    NetworkController(const NetworkController&) = delete;
    NetworkController& operator=(const NetworkController&) = delete;
    
    // === Network Lifecycle Management ===
    
    /**
     * @brief Initialize EtherCAT master on the specified interface
     * @return true if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Scan network for EtherCAT slaves
     * @return true if scan completed (may find 0 slaves)
     */
    bool scanNetwork();
    
    /**
     * @brief Configure all detected slaves (PDO mapping, etc.)
     * @return true if all slaves configured successfully
     */
    bool configureSlaves();
    
    /**
     * @brief Start network operation (transition to operational state)
     * @return true if network reached operational state
     */
    bool startOperation();
    
    /**
     * @brief Stop network operation and cleanup resources
     */
    void stopOperation();
    
    // === Real-Time Communication ===
    
    /**
     * @brief Perform one communication cycle (send/receive process data)
     * @return Working counter from EtherCAT communication
     * 
     * This function handles the real-time EtherCAT communication cycle.
     * Should be called at the configured frequency.
     */
    int performCommunicationCycle();
    
    /**
     * @brief Check if network is operational and ready for motor control
     * @return true if network is operational
     */
    bool isOperational() const;
    
    // === Multi-Motor PDO Access ===
    
    /**
     * @brief Get output PDO pointer for specific slave
     * @param slave_index EtherCAT slave index (1-based)
     * @return Pointer to output PDO or nullptr if invalid
     */
    OutputPDO* getOutputPDO(int slave_index);
    
    /**
     * @brief Get input PDO pointer for specific slave
     * @param slave_index EtherCAT slave index (1-based)  
     * @return Pointer to input PDO or nullptr if invalid
     */
    const InputPDO* getInputPDO(int slave_index) const;
    
    // === Network Information ===
    
    /**
     * @brief Get network status information
     * @return NetworkStatus structure with current network state
     */
    NetworkStatus getNetworkStatus() const;
    
    /**
     * @brief Get information about all detected slaves
     * @return Vector of SlaveInfo structures
     */
    std::vector<SlaveInfo> getSlaveList() const;
    
    /**
     * @brief Get number of detected EtherCAT slaves
     * @return Number of slaves found during network scan
     */
    int getSlaveCount() const;
    
    /**
     * @brief Get slave information for specific index
     * @param slave_index EtherCAT slave index (1-based)
     * @return SlaveInfo structure or invalid info if not found
     */
    SlaveInfo getSlaveInfo(int slave_index) const;
    
    // === Motor Detection and Management ===
    
    /**
     * @brief Get detected motor information for specific slave
     * @param slave_index EtherCAT slave index (1-based)
     * @return DetectedMotorInfo with motor type and configuration
     */
    DetectedMotorInfo getDetectedMotorInfo(int slave_index) const;
    
    /**
     * @brief Get all detected motor information
     * @return Vector of DetectedMotorInfo for all slaves
     */
    std::vector<DetectedMotorInfo> getAllDetectedMotors() const;
    
    /**
     * @brief Get motor manager instance
     * @return Pointer to motor manager (for advanced usage)
     */
    const MotorManager* getMotorManager() const;
    
    // === State Monitoring ===
    
    /**
     * @brief Check all slave states and network health
     * @return true if all slaves are in correct states
     */
    bool checkSlaveStates();
    
    /**
     * @brief Check for communication errors
     * @return true if communication errors detected
     */
    bool hasDataErrors() const;
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;
    
    // === Frequency Configuration ===
    
    /**
     * @brief Get current update frequency in Hz
     * @return Update frequency in Hz
     */
    double getUpdateFrequencyHz() const;
    
    /**
     * @brief Get cycle time in microseconds
     * @return Cycle time in microseconds for timing
     */
    int getCycleTimeUs() const;
    
    /**
     * @brief Get cycle time in milliseconds
     * @return Cycle time in milliseconds
     */
    double getCycleTimeMs() const;
    
private:
    // === Network Configuration ===
    std::string interface_name_;           ///< EtherCAT network interface name
    double update_frequency_hz_;           ///< Network update frequency in Hz  
    int cycle_time_us_;                   ///< Cycle time in microseconds
    double cycle_time_ms_;                ///< Cycle time in milliseconds
    
    // === Network State ===
    std::atomic<bool> initialized_;        ///< EtherCAT master initialized
    std::atomic<bool> operational_;        ///< Network operational state
    std::atomic<int> slave_count_;         ///< Number of detected slaves
    std::atomic<int> expected_wkc_;        ///< Expected working counter
    std::atomic<int> actual_wkc_;          ///< Last working counter
    std::atomic<uint64_t> cycle_count_;    ///< Total cycles completed
    
    // === EtherCAT Resources ===
    std::vector<uint8_t> iomap_;          ///< EtherCAT I/O mapping buffer (replaces static)
    std::vector<SlaveInfo> slaves_;       ///< Information about all slaves
    mutable std::mutex slaves_mutex_;     ///< Thread safety for slave access
    
    // === Error Handling ===
    mutable std::mutex error_mutex_;      ///< Thread safety for error handling
    std::string last_error_;              ///< Last error message
    std::atomic<bool> has_errors_;        ///< Error flag
    
    // === Timing ===
    std::chrono::high_resolution_clock::time_point last_cycle_time_;  ///< Last cycle timestamp
    
    // === Motor Management ===
    std::unique_ptr<MotorManager> motor_manager_;  ///< Motor type detection and configuration
    
    // === Internal Helper Functions ===
    
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
     * @brief Initialize slave information after network scan
     */
    void initializeSlaveInfo();
    
    /**
     * @brief Setup PDO pointers for all slaves
     * @return true if PDO setup successful
     */
    bool setupPDOPointers();
    
    /**
     * @brief Validate slave index
     * @param slave_index Index to validate
     * @return true if valid slave index
     */
    bool isValidSlaveIndex(int slave_index) const;
};

} // namespace synapticon_motor