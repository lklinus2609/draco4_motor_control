/**
 * @file motor_controller.hpp
 * @brief EtherCAT Motor Controller Interface
 * 
 * This header defines the MotorController class for controlling Synapticon servo motors
 * via EtherCAT communication using the SOEM library. The controller implements
 * the CIA402 state machine for motor operation and provides velocity, position,
 * and torque control modes.
 * 
 * @note Real-time operation at configurable frequency (default: 250Hz, 4ms cycle time)
 */

#pragma once

// Debug logging control - set to 0 for production builds
#define MOTOR_DEBUG_LOGGING 0

#include "motor_pdo_structures.hpp"
#include "motor_configuration.hpp"
#include "motor_sdo_manager.hpp"

// Forward declaration to avoid circular dependency
namespace synapticon_motor {
    class NetworkController;
}
#include <soem/ethercat.h>
#include <string>
#include <atomic>
#include <chrono>
#include <memory>

namespace synapticon_motor {

/**
 * @class MotorController
 * @brief EtherCAT motor controller for Synapticon servo motors
 * 
 * Provides high-level interface for controlling Synapticon servo motors over EtherCAT.
 * Implements CIA402 state machine, supports multiple control modes, and includes
 * safety features like torque ramping and fault recovery.
 */
class MotorController {
public:
    /**
     * @brief Available motor control modes
     */
    enum ControlMode {
        VELOCITY_MODE = MotorConstants::VELOCITY_MODE,  ///< Velocity control mode
        POSITION_MODE = MotorConstants::POSITION_MODE,  ///< Position control mode
        TORQUE_MODE = MotorConstants::TORQUE_MODE       ///< Torque control mode
    };
    
    /**
     * @brief Motor state according to CIA402 state machine
     */
    enum MotorState {
        NOT_READY,              ///< Motor not ready for operation
        READY_TO_SWITCH_ON,     ///< Ready to switch on
        SWITCHED_ON,            ///< Motor switched on but not enabled
        OPERATION_ENABLED,      ///< Motor enabled and operational
        FAULT,                  ///< Motor in fault state
        UNKNOWN                 ///< Unknown state
    };
    
    /**
     * @brief Error message severity levels
     */
    enum class ErrorSeverity { INFO, WARNING, ERROR, CRITICAL };
    
    /**
     * @brief Constructor
     * @param network Shared pointer to NetworkController managing EtherCAT communication
     * @param slave_index EtherCAT slave index on the network (1-based)
     * @param update_frequency_hz Communication update frequency in Hz (default: 250.0)
     */
    MotorController(std::shared_ptr<NetworkController> network, 
                   int slave_index, 
                   double update_frequency_hz = MotorConstants::DEFAULT_RT_FREQUENCY_HZ);
    
    
    /**
     * @brief Destructor - automatically stops operation
     */
    ~MotorController();
    
    // === Motor Operations ===
    // Note: Network operations (initialize, scan, configure, start, stop) 
    // are handled by NetworkController. MotorController focuses on motor control only.
    
    // === Configuration Management ===
    
    /**
     * @brief Load motor configuration from CSV file
     * @param config_file Path to configuration CSV file
     * @return true if configuration loaded successfully
     */
    bool loadConfig(const std::string& config_file);
    
    /**
     * @brief Get current configuration parser
     * @return Pointer to configuration parser, nullptr if not loaded
     */
    const MotorConfigParser* getConfig() const;
    
    /**
     * @brief Check if configuration is loaded
     * @return true if configuration is available
     */
    bool hasConfig() const;
    
    /**
     * @brief Upload configuration parameters to motor via SDO
     * @return true if upload successful
     */
    bool uploadConfig();
    
    /**
     * @brief Upload only critical parameters (gains + safety limits)
     * @return true if upload successful
     */
    bool uploadCriticalParam();
    
    /**
     * @brief Get SDO manager for advanced operations
     * @return Pointer to SDO manager, nullptr if not available
     */
    const MotorSDOManager* getSDO() const;
    
    // === Motor Control ===
    
    /**
     * @brief Enable motor using CIA402 state machine
     * @return true if motor successfully enabled
     */
    bool enable_motor();
    
    /**
     * @brief Disable motor and stop all motion
     * @return true if motor successfully disabled
     */
    bool disable_motor();
    
    /**
     * @brief Check if motor is enabled
     * @return true if motor is enabled
     */
    bool is_motor_enabled() const;
    
    // === Control Commands ===
    
    /**
     * @brief Set velocity command in RPM
     * @param rpm Target velocity in revolutions per minute
     * @return true if command accepted
     */
    bool set_velocity_rpm(int rpm);
    
    /**
     * @brief Set position command in encoder counts
     * @param position Target position in encoder counts
     * @return true if command accepted
     */
    bool set_position_counts(int32_t position);
    
    /**
     * @brief Set torque command in millinewton-meters
     * @param torque Target torque in mNm
     * @return true if command accepted
     */
    bool set_torque_millinm(int16_t torque);
    
    // === Feedback ===
    
    /**
     * @brief Get actual velocity feedback (motor shaft)
     * @return Current motor shaft velocity in RPM (double precision)
     */
    double getMotorRPM() const;
    
    /**
     * @brief Get actual output shaft velocity feedback
     * @return Current output shaft velocity in RPM (accounts for configured gear reduction)
     */
    double getOutputRPM() const;
    
    /**
     * @brief Get actual position feedback (motor shaft)
     * @return Current motor shaft position in encoder counts
     */
    int32_t getPosition() const;
    
    /**
     * @brief Get actual output shaft position feedback
     * @return Current output shaft position in encoder counts (accounts for configured gear reduction)
     */
    int32_t getOutputPos() const;
    
    /**
     * @brief Get actual torque feedback
     * @return Current torque in mNm
     */
    int16_t getTorque() const;
    
    /**
     * @brief Get raw status word from motor
     * @return CIA402 status word
     */
    uint16_t getStatus() const;
    
    // === State and Diagnostics ===
    
    /**
     * @brief Get current motor state
     * @return Current MotorState enum value
     */
    MotorState get_motor_state() const;
    
    /**
     * @brief Get human-readable motor state string
     * @return String description of current state
     */
    const char* getStateStr() const;
    
    /**
     * @brief Check if motor has fault condition
     * @return true if fault detected
     */
    bool has_fault() const;
    
    /**
     * @brief Clear motor faults
     * @return true if fault reset command sent
     */
    bool clear_faults();
    
    /**
     * @brief Update motor communication (call at configured frequency intervals)
     * 
     * Handles EtherCAT communication, torque ramping, and fault recovery.
     * Should be called regularly in a real-time loop at the configured frequency.
     * Use getCycleTimeUs() to get the correct sleep interval.
     */
    void update();
    
    // === Frequency Configuration ===
    
    /**
     * @brief Get current update frequency in Hz
     * @return Update frequency in Hz
     */
    double getUpdateFrequencyHz() const;
    
    /**
     * @brief Get cycle time in microseconds
     * @return Cycle time in microseconds for sleep timing
     */
    int getCycleTimeUs() const;
    
    /**
     * @brief Get cycle time in milliseconds  
     * @return Cycle time in milliseconds
     */
    double getCycleTimeMs() const;
    
    // === Mode Control ===
    
    /**
     * @brief Set control mode
     * @param mode Desired control mode
     * @return true if mode change accepted
     */
    bool set_control_mode(ControlMode mode);
    
    /**
     * @brief Get current control mode
     * @return Current ControlMode
     */
    ControlMode get_current_mode() const;
    
    // === Safety Functions ===
    
    /**
     * @brief Emergency stop - immediately halt all motion
     */
    void eStop();
    
    /**
     * @brief Check if controller is in fault recovery mode
     * @return true if fault recovery active
     */
    bool is_in_fault_recovery() const;
    
private:
    // === Internal Helper Functions ===
    
    /**
     * @brief Execute one step of motor enable sequence
     * @return Current enable step number
     */
    int enable_motor_sequence();
    
    /**
     * @brief Log error message with severity level
     * @param severity Error severity level
     * @param message Error message text
     */
    void log_error(ErrorSeverity severity, const std::string& message);
    
    /**
     * @brief Validate torque command before execution
     * @param torque Torque command to validate
     * @return true if command is valid
     */
    bool validate_torque_command(int16_t torque);
    
    
    /**
     * @brief Apply torque ramping for smooth control
     */
    void ramp_torque_command();
    
    /**
     * @brief Apply position updates for continuous control
     */
    void update_position_command();
    
    /**
     * @brief Apply velocity updates for continuous control
     */
    void update_velocity_command();
    
    /**
     * @brief Handle fault recovery by ramping torque to zero
     */
    void handle_fault_recovery();
    
    /**
     * @brief Check EtherCAT slave states
     * @return true if all slaves are in correct state
     */
    bool check_slave_states();
    
    // === Member Variables ===
    
    // Network dependency
    std::shared_ptr<NetworkController> network_;  ///< Network controller managing EtherCAT communication
    int slave_index_;                            ///< EtherCAT slave index on the network
    
    // Motor state
    std::atomic<bool> motor_enabled_;   ///< Motor enabled state (thread-safe)
    ControlMode current_mode_;          ///< Current control mode
    int enable_step_;                   ///< Current step in enable sequence
    
    // Torque control state
    int16_t current_torque_command_;   ///< Current ramped torque command (mNm)
    int16_t target_torque_command_;    ///< Target torque command (mNm)
    bool fault_recovery_active_;       ///< Fault recovery ramping active flag
    
    // Position control state
    int32_t target_position_command_;  ///< Target position command (counts)
    int32_t current_position_command_; ///< Current ramped position command (counts)
    
    // Velocity control state  
    int target_velocity_command_;      ///< Target velocity command (RPM)
    int current_velocity_command_;     ///< Current ramped velocity command (RPM)
    
    // Position control rate limiting
    int32_t last_position_command_;    ///< Last position command for rate limiting
    std::chrono::steady_clock::time_point last_position_time_;  ///< Timestamp of last position command
    
    // Configuration management
    std::unique_ptr<MotorConfigParser> config_parser_;  ///< Motor configuration parser
    std::unique_ptr<MotorSDOManager> sdo_manager_;      ///< SDO communication manager
    
    // Frequency configuration
    double update_frequency_hz_;     ///< Communication update frequency in Hz
    int cycle_time_us_;             ///< Cycle time in microseconds
    double cycle_time_ms_;          ///< Cycle time in milliseconds
};

} // namespace synapticon_motor
