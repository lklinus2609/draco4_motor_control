/**
 * @file motor_controller.cpp
 * @brief Implementation of MotorController class
 * 
 * Provides EtherCAT-based control for Synapticon motors using the SOEM library.
 * Implements CIA402 state machine and supports velocity, position, and torque control.
 */

#include "motor_controller.hpp"
#include "network_controller.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <stdexcept>

namespace synapticon_motor {

MotorController::MotorController(std::shared_ptr<NetworkController> network, 
                               int slave_index, 
                               double update_frequency_hz) 
    : network_(network), slave_index_(slave_index),
      motor_enabled_(false), current_mode_(VELOCITY_MODE), enable_step_(0),
      current_torque_command_(0), target_torque_command_(0), fault_recovery_active_(false),
      target_position_command_(0), current_position_command_(0),
      target_velocity_command_(0), current_velocity_command_(0),
      last_position_command_(0),
      update_frequency_hz_(update_frequency_hz),
      cycle_time_us_(static_cast<int>(1000000.0 / update_frequency_hz)),
      cycle_time_ms_(1000.0 / update_frequency_hz) {
    
    last_position_time_ = std::chrono::steady_clock::now();
    
    if (!network_) {
        throw std::runtime_error("NetworkController cannot be null");
    }
    
    std::cout << "MotorController created for slave " << slave_index_ 
              << " at " << update_frequency_hz_ << "Hz" << std::endl;
}


MotorController::~MotorController() {
    // Network cleanup is handled by NetworkController
    // Motor-specific cleanup only
    motor_enabled_.store(false);
}


void MotorController::update() {
    if (network_ && network_->isOperational()) {
        // Motor-specific control logic
        // Execute control commands based on active mode (mutually exclusive)
        if (current_mode_ == TORQUE_MODE || fault_recovery_active_) {
            ramp_torque_command();
        } else if (current_mode_ == POSITION_MODE) {
            update_position_command();
        } else if (current_mode_ == VELOCITY_MODE) {
            update_velocity_command();
        }
        
        if (has_fault() && !fault_recovery_active_) {
            handle_fault_recovery();
        }
    }
}

bool MotorController::enable_motor() {
    std::cout << "Starting motor enable sequence..." << std::endl;
    enable_step_ = 0;
    motor_enabled_ = false;
    
    // Precise timing for enable sequence using configured frequency
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(cycle_time_us_);
    
    for (int i = 0; i < 1000 && !motor_enabled_; i++) {
        update();
        
        if (i % 10 == 0) {
            enable_step_ = enable_motor_sequence();
        }
        
        if (i % 100 == 0 && network_) {
            const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
            if (input_pdo) {
                std::cout << "Enable cycle " << i << ": Status=0x" << std::hex << input_pdo->statusword 
                          << ", State=" << getStateStr() << std::dec << std::endl;
            }
        }
        
        // Sleep until next cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(cycle_time_us_);
    }
    
    std::cout << "Enable sequence completed. Motor enabled: " << (motor_enabled_ ? "YES" : "NO") << std::endl;
    return motor_enabled_;
}

bool MotorController::disable_motor() {
    motor_enabled_ = false;
    fault_recovery_active_ = false;
    current_torque_command_ = 0;
    target_torque_command_ = 0;
    target_velocity_command_ = 0;
    target_position_command_ = 0;
    
    if (network_) {
        OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
        if (output_pdo) {
            output_pdo->controlword = MotorConstants::CONTROLWORD_SHUTDOWN;
            output_pdo->target_velocity = 0;
            output_pdo->target_torque = 0;
            output_pdo->target_position = 0;
        }
    }
    return true;
}

bool MotorController::set_velocity_rpm(int rpm) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    // Get max velocity from config, use default if no config available
    uint32_t max_velocity = config_parser_ ? config_parser_->getMaxVelocityRPM() : MotorConstants::DEFAULT_MAX_VELOCITY_RPM;
    
    if (abs(rpm) > static_cast<int>(max_velocity)) {
        log_error(ErrorSeverity::ERROR, 
                  "Velocity " + std::to_string(rpm) + " exceeds safety limit of " + 
                  std::to_string(max_velocity) + " RPM");
        return false;
    }
    
    target_velocity_command_ = rpm;
    
    if (current_mode_ != VELOCITY_MODE && network_) {
        OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
        if (output_pdo) {
            // Clear position commands when switching to velocity mode
            target_position_command_ = getPosition();
            current_position_command_ = target_position_command_;
            output_pdo->target_position = static_cast<uint32_t>(target_position_command_);
            
            // Set velocity mode
            output_pdo->modes_of_operation = MotorConstants::VELOCITY_MODE;
            output_pdo->controlword = MotorConstants::CONTROLWORD_VELOCITY_MODE;
            current_mode_ = VELOCITY_MODE;
            
            std::cout << "Mode switched to VELOCITY_MODE, position commands cleared" << std::endl;
        }
    }
    
    return true;
}

bool MotorController::set_position_counts(int32_t position) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    // GEAR RATIO ANALYSIS: PDO goes to motor shaft, but user commands output shaft position
    // To achieve desired output shaft position, motor shaft must move gear_ratio times more
    double gear_ratio = MotorConstants::getGearReductionRatio(config_parser_.get());
    int32_t motor_shaft_position = static_cast<int32_t>(position * gear_ratio);
    
    target_position_command_ = motor_shaft_position;
    
    // Debug logging for position command scaling
#if MOTOR_DEBUG_LOGGING
    std::cout << "[POS] target_output_shaft=" << position 
              << " → motor_shaft_scaled=" << motor_shaft_position
              << " (×" << gear_ratio << ")" << std::endl;
#endif
    
    if (current_mode_ != POSITION_MODE && network_) {
        OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
        if (output_pdo) {
            // Clear velocity commands when switching to position mode
            target_velocity_command_ = 0;
            current_velocity_command_ = 0;
            output_pdo->target_velocity = 0;
            
            // Set position mode
            output_pdo->modes_of_operation = MotorConstants::POSITION_MODE;
            output_pdo->controlword = MotorConstants::CONTROLWORD_POSITION_MODE;
            current_mode_ = POSITION_MODE;
            current_position_command_ = getPosition();  // This is already motor shaft position
            
            std::cout << "Mode switched to POSITION_MODE, velocity commands cleared" << std::endl;
        }
    }
    
    return true;
}

bool MotorController::set_torque_millinm(int16_t torque) {
    if (!validate_torque_command(torque)) {
        return false;
    }
    
    target_torque_command_ = MotorConstants::clamp_torque(torque, config_parser_.get());
    
    if (target_torque_command_ != torque) {
        log_error(ErrorSeverity::WARNING, 
                  "Torque " + std::to_string(torque) + " clamped to " + 
                  std::to_string(target_torque_command_) + " mNm");
    }
    
    if (current_mode_ != TORQUE_MODE && network_) {
        OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
        if (output_pdo) {
            // Clear other mode commands when switching to torque mode
            target_velocity_command_ = 0;
            current_velocity_command_ = 0;
            output_pdo->target_velocity = 0;
            
            target_position_command_ = getPosition();
            current_position_command_ = target_position_command_;
            output_pdo->target_position = static_cast<uint32_t>(target_position_command_);
            
            // Set torque mode
            output_pdo->modes_of_operation = MotorConstants::TORQUE_MODE;
            output_pdo->controlword = MotorConstants::CONTROLWORD_ENABLE_OPERATION;
            current_mode_ = TORQUE_MODE;
            
            std::cout << "Mode switched to TORQUE_MODE, other commands cleared" << std::endl;
        }
    }
    
    return true;
}

const char* MotorController::getStateStr() const {
    if (!network_) return "NO_DATA";
    
    const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
    if (!input_pdo) return "NO_DATA";
    
    uint16_t statusword = input_pdo->statusword;
    
    if (statusword & MotorConstants::STATUSWORD_FAULT)
        return "FAULT";
    if ((statusword & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_OPERATION_ENABLED_VALUE)
        return "OPERATION ENABLED";
    if ((statusword & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_SWITCHED_ON)
        return "SWITCHED ON";
    if ((statusword & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_READY_TO_SWITCH_ON)
        return "READY TO SWITCH ON";
    if ((statusword & MotorConstants::STATUSWORD_SWITCH_DISABLED_MASK) == MotorConstants::STATUSWORD_SWITCH_ON_DISABLED_VALUE)
        return "SWITCH ON DISABLED";
    return "UNKNOWN";
}

double MotorController::getMotorRPM() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        if (input_pdo) {
            // Calculate and log all scaling steps
            uint32_t raw_pdo = input_pdo->velocity_actual;
            int32_t signed_pdo = static_cast<int32_t>(raw_pdo);
            double motor_shaft_rpm = static_cast<double>(signed_pdo) / 1000.0;
            double gear_ratio = MotorConstants::getGearReductionRatio(config_parser_.get());
            double output_shaft_rpm = motor_shaft_rpm / gear_ratio;
            
            // Debug logging for velocity feedback scaling
#if MOTOR_DEBUG_LOGGING
            std::cout << "[FB] raw_pdo_uint32=" << raw_pdo
                      << " → signed_int32=" << signed_pdo  
                      << " → motor_shaft_rpm=" << motor_shaft_rpm
                      << " → output_shaft_rpm=" << output_shaft_rpm
                      << " → returning_motor_shaft=" << motor_shaft_rpm << std::endl;
#endif
            
            // Return output shaft RPM for now (we can add separate function for output shaft if needed)
            return output_shaft_rpm;
        }
    }
    return 0.0;
}

double MotorController::getOutputRPM() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        if (input_pdo) {
            uint32_t raw_pdo = input_pdo->velocity_actual;
            int32_t signed_pdo = static_cast<int32_t>(raw_pdo);
            double motor_shaft_rpm = static_cast<double>(signed_pdo) / 1000.0;
            double gear_ratio = MotorConstants::getGearReductionRatio(config_parser_.get());
            return motor_shaft_rpm / gear_ratio;
        }
    }
    return 0.0;
}

int32_t MotorController::getPosition() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        if (input_pdo) {
            return input_pdo->position_actual;  // Returns motor shaft position
        }
    }
    return 0;
}

int32_t MotorController::getOutputPos() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        if (input_pdo) {
            int32_t motor_shaft_position = input_pdo->position_actual;
            double gear_ratio = MotorConstants::getGearReductionRatio(config_parser_.get());
            return static_cast<int32_t>(motor_shaft_position / gear_ratio);
        }
    }
    return 0;
}

int16_t MotorController::getTorque() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        if (input_pdo) {
            return MotorConstants::pdo_to_millinm(input_pdo->torque_actual, config_parser_.get());
        }
    }
    return 0;
}

uint16_t MotorController::getStatus() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        if (input_pdo) {
            return input_pdo->statusword;
        }
    }
    return 0;
}

bool MotorController::is_motor_enabled() const {
    return motor_enabled_;
}

bool MotorController::has_fault() const {
    if (network_) {
        const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
        return input_pdo && (input_pdo->statusword & MotorConstants::STATUSWORD_FAULT);
    }
    return false;
}

bool MotorController::clear_faults() {
    if (network_) {
        OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
        if (output_pdo) {
            output_pdo->controlword = MotorConstants::CONTROLWORD_FAULT_RESET;
            fault_recovery_active_ = false;
            return true;
        }
    }
    return false;
}

MotorController::MotorState MotorController::get_motor_state() const {
    if (!network_) return UNKNOWN;
    
    const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
    if (!input_pdo) return UNKNOWN;
    
    uint16_t statusword = input_pdo->statusword;
    
    if (statusword & MotorConstants::STATUSWORD_FAULT)
        return FAULT;
    if ((statusword & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_OPERATION_ENABLED_VALUE)  // Operation enabled
        return OPERATION_ENABLED;
    if ((statusword & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_SWITCHED_ON)
        return SWITCHED_ON;
    if ((statusword & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_READY_TO_SWITCH_ON)
        return READY_TO_SWITCH_ON;
    
    return NOT_READY;
}

bool MotorController::set_control_mode(ControlMode mode) {
    current_mode_ = mode;
    return true;  // Mode is set when command is sent
}

MotorController::ControlMode MotorController::get_current_mode() const {
    return current_mode_;
}

void MotorController::eStop() {
    log_error(ErrorSeverity::CRITICAL, "Emergency stop activated");
    
    fault_recovery_active_ = true;
    target_torque_command_ = 0;
    
    if (network_) {
        OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
        if (output_pdo) {
            output_pdo->target_velocity = 0;
            output_pdo->controlword = MotorConstants::CONTROLWORD_QUICK_STOP;
        }
    }
}

bool MotorController::is_in_fault_recovery() const {
    return fault_recovery_active_;
}

int MotorController::enable_motor_sequence() {
    if (!network_) return enable_step_;
    
    const InputPDO* input_pdo = network_->getInputPDO(slave_index_);
    OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
    
    if (!input_pdo || !output_pdo) return enable_step_;
    
    uint16_t status = input_pdo->statusword;
    
    switch(enable_step_) {
        case 0:
            if (status & MotorConstants::STATUSWORD_FAULT) {
                output_pdo->controlword = MotorConstants::CONTROLWORD_FAULT_RESET;
                return 0;
            }
            return 1;
            
        case 1: {
            output_pdo->controlword = MotorConstants::CONTROLWORD_SHUTDOWN;
            network_->flushControlWordUpdate(slave_index_);
            
            uint16_t masked_status = status & MotorConstants::STATUSWORD_STATE_MASK;
            if (masked_status == MotorConstants::STATUSWORD_READY_TO_SWITCH_ON) {
                return 2;
            }
            return 1;
        }
            
        case 2: {
            output_pdo->controlword = MotorConstants::CONTROLWORD_SWITCH_ON;
            output_pdo->modes_of_operation = MotorConstants::VELOCITY_MODE;
            network_->flushControlWordUpdate(slave_index_);
            
            if ((status & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_SWITCHED_ON) {
                return 3;
            }
            return 2;
        }
            
        case 3: {
            output_pdo->controlword = MotorConstants::CONTROLWORD_ENABLE_OPERATION;
            output_pdo->modes_of_operation = MotorConstants::VELOCITY_MODE;
            network_->flushControlWordUpdate(slave_index_);
            
            if ((status & MotorConstants::STATUSWORD_STATE_MASK) == MotorConstants::STATUSWORD_OPERATION_ENABLED_VALUE) {
                motor_enabled_ = true;
                return 4;
            }
            return 3;
        }
            
        default:
            return 4;
    }
}

void MotorController::log_error(ErrorSeverity severity, const std::string& message) {
    std::string prefix;
    switch(severity) {
        case ErrorSeverity::INFO: prefix = "INFO"; break;
        case ErrorSeverity::WARNING: prefix = "WARN"; break;
        case ErrorSeverity::ERROR: prefix = "ERROR"; break;
        case ErrorSeverity::CRITICAL: prefix = "CRITICAL"; break;
    }
    std::cout << "[" << prefix << "] " << message << std::endl;
}

bool MotorController::validate_torque_command(int16_t torque) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    if (has_fault()) {
        log_error(ErrorSeverity::ERROR, "Motor fault detected, cannot set torque");
        return false;
    }
    
    int16_t max_torque = MotorConstants::getMaxTorqueMNm(config_parser_.get());
    if (abs(torque) > max_torque) {
        log_error(ErrorSeverity::WARNING, 
                  "Torque " + std::to_string(torque) + " exceeds limit ±" + 
                  std::to_string(max_torque) + " mNm");
    }
    
    return true;
}

void MotorController::ramp_torque_command() {
    if (!network_) return;
    
    OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
    if (!output_pdo) return;
    
    int16_t torque_diff = target_torque_command_ - current_torque_command_;
    
    if (torque_diff == 0) {
        return;
    }
    
    int16_t torque_ramp_rate = MotorConstants::getTorqueRampRate(config_parser_.get());
    int16_t ramp_step = torque_ramp_rate;
    if (torque_diff < 0) {
        ramp_step = -ramp_step;
    }
    
    if (abs(torque_diff) <= torque_ramp_rate) {
        current_torque_command_ = target_torque_command_;
    } else {
        current_torque_command_ += ramp_step;
    }
    
    uint16_t pdo_torque = MotorConstants::millinm_to_pdo(current_torque_command_, config_parser_.get());
    output_pdo->target_torque = pdo_torque;
    
    if (fault_recovery_active_ && current_torque_command_ == 0) {
        fault_recovery_active_ = false;
        log_error(ErrorSeverity::INFO, "Fault recovery complete - torque ramped to zero");
    }
}

void MotorController::update_position_command() {
    if (!network_) return;
    
    OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
    if (!output_pdo) return;
    
    int32_t position_diff = target_position_command_ - current_position_command_;
    
    if (position_diff == 0) {
        return;
    }
    
    int32_t max_step = MotorConstants::getMaxPositionChangePerCycle(config_parser_.get());
    int32_t step = position_diff;
    
    if (abs(position_diff) > max_step) {
        step = (position_diff > 0) ? max_step : -max_step;
    }
    
    current_position_command_ += step;
    output_pdo->target_position = static_cast<uint32_t>(current_position_command_);
    output_pdo->controlword = MotorConstants::CONTROLWORD_POSITION_MODE;
}

void MotorController::update_velocity_command() {
    if (!network_) return;
    
    OutputPDO* output_pdo = network_->getOutputPDO(slave_index_);
    if (!output_pdo) return;
    
    current_velocity_command_ = target_velocity_command_;
    
    // GEAR RATIO ANALYSIS: PDO goes to motor shaft, but user commands output shaft velocity
    // To achieve desired output shaft RPM, motor shaft must run gear_ratio times faster
    double gear_ratio = MotorConstants::getGearReductionRatio(config_parser_.get());
    double motor_shaft_rpm = current_velocity_command_ * gear_ratio;
    
    // Calculate and log all scaling steps
    uint32_t raw_command = static_cast<uint32_t>(current_velocity_command_);
    uint32_t gear_scaled_command = static_cast<uint32_t>(motor_shaft_rpm);
    uint32_t final_scaled_command = static_cast<uint32_t>(motor_shaft_rpm * 1000);
    
    output_pdo->target_velocity = final_scaled_command;
    
    // Debug logging for velocity command scaling
#if MOTOR_DEBUG_LOGGING
    std::cout << "[CMD] target_output_shaft_rpm=" << target_velocity_command_ 
              << " → motor_shaft_rpm=" << motor_shaft_rpm
              << " → raw_uint32=" << raw_command
              << " → gear_scaled=" << gear_scaled_command
              << " → final_1000x=" << final_scaled_command
              << " → sent_to_pdo=" << output_pdo->target_velocity << std::endl;
#endif
    
    // Only update control word if motor is enabled and we're in velocity mode
    if (motor_enabled_ && current_mode_ == VELOCITY_MODE) {
        output_pdo->controlword = MotorConstants::CONTROLWORD_ENABLE_OPERATION;
    }
}

void MotorController::handle_fault_recovery() {
    log_error(ErrorSeverity::WARNING, "Fault detected - initiating torque ramp down");
    
    fault_recovery_active_ = true;
    target_torque_command_ = 0;
}

// === Configuration Management Implementation ===

bool MotorController::loadConfig(const std::string& config_file) {
    std::cout << "Loading motor configuration from: " << config_file << std::endl;
    
    // Create new configuration parser
    config_parser_ = std::make_unique<MotorConfigParser>();
    
    // Parse configuration file
    bool loaded = config_parser_->parseCSV(config_file);
    
    if (!loaded) {
        std::cerr << "Failed to load configuration file" << std::endl;
        for (const auto& error : config_parser_->getErrors()) {
            std::cerr << "  - " << error << std::endl;
        }
        config_parser_.reset();
        return false;
    }
    
    // Validate critical parameters
    auto motor_specs = config_parser_->getMotorSpecs();
    auto control_gains = config_parser_->getControlGains();
    
    std::cout << "Configuration loaded successfully" << std::endl;
    std::cout << "  Parameters: " << config_parser_->getParameterCount() << std::endl;
    std::cout << "  Encoder Resolution: " << motor_specs.encoder_resolution << " counts/rev" << std::endl;
    // Velocity resolution and scaling are now configuration-driven
    // Available via config_parser_->getVelocityResolution() and calcVelScale()
    
    return true;
}

const MotorConfigParser* MotorController::getConfig() const {
    return config_parser_.get();
}

bool MotorController::hasConfig() const {
    return config_parser_ && config_parser_->isLoaded();
}

bool MotorController::uploadConfig() {
    if (!hasConfig()) {
        log_error(ErrorSeverity::ERROR, "No configuration loaded - cannot upload parameters");
        return false;
    }
    
    if (!network_ || !network_->isOperational()) {
        log_error(ErrorSeverity::ERROR, "EtherCAT not operational - cannot upload parameters");
        return false;
    }
    
    // Create SDO manager if not already created
    if (!sdo_manager_) {
        sdo_manager_ = std::make_unique<MotorSDOManager>(slave_index_);
        std::cout << "SDO manager created for slave " << slave_index_ << std::endl;
    }
    
    // Upload complete configuration
    MotorSDOManager::SDOResult result = sdo_manager_->uploadComplete(*config_parser_);
    
    if (result == MotorSDOManager::SDOResult::SUCCESS) {
        std::cout << "Motor configuration uploaded successfully!" << std::endl;
        
        // Print upload statistics
        auto stats = sdo_manager_->getStatistics();
        std::cout << "Upload Statistics:" << std::endl;
        std::cout << "  Total uploads: " << stats.uploads_attempted << std::endl;
        std::cout << "  Successful: " << stats.uploads_successful << std::endl;
        std::cout << "  Failed: " << stats.uploads_failed << std::endl;
        std::cout << "  Success rate: " << (100.0 * stats.uploads_successful / stats.uploads_attempted) << "%" << std::endl;
        std::cout << "  Average upload time: " << stats.average_upload_time_ms << " ms" << std::endl;
        
        return true;
    } else {
        std::cerr << "Configuration upload failed: " << sdo_manager_->getLastError() << std::endl;
        return false;
    }
}

bool MotorController::uploadCriticalParam() {
    if (!hasConfig()) {
        log_error(ErrorSeverity::ERROR, "No configuration loaded - cannot upload parameters");
        return false;
    }
    
    if (!network_ || !network_->isOperational()) {
        log_error(ErrorSeverity::ERROR, "EtherCAT not operational - cannot upload parameters");
        return false;
    }
    
    // Create SDO manager if not already created
    if (!sdo_manager_) {
        sdo_manager_ = std::make_unique<MotorSDOManager>(slave_index_);
        std::cout << "SDO manager created for slave " << slave_index_ << std::endl;
    }
    
    // Upload only critical parameters (faster, safer)
    MotorSDOManager::SDOResult result = sdo_manager_->uploadCriticalParameters(*config_parser_);
    
    if (result == MotorSDOManager::SDOResult::SUCCESS) {
        std::cout << "Critical parameters uploaded successfully!" << std::endl;
        return true;
    } else {
        std::cerr << "Critical parameter upload failed: " << sdo_manager_->getLastError() << std::endl;
        return false;
    }
}

const MotorSDOManager* MotorController::getSDO() const {
    return sdo_manager_.get();
}

double MotorController::getUpdateFrequencyHz() const {
    return update_frequency_hz_;
}

int MotorController::getCycleTimeUs() const {
    return cycle_time_us_;
}

double MotorController::getCycleTimeMs() const {
    return cycle_time_ms_;
}


} // namespace synapticon_motor
