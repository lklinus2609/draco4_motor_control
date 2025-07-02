/**
 * @file network_controller.cpp
 * @brief EtherCAT Network Controller Implementation
 * 
 * Implementation of NetworkController class for managing EtherCAT networks
 * with multiple motor controllers. Extracted from MotorController to enable
 * multi-motor and multi-network architectures.
 */

#include "network_controller.hpp"
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <cstring>
#include <thread>

namespace synapticon_motor {

NetworkController::NetworkController(const std::string& interface_name, double update_frequency_hz)
    : interface_name_(interface_name),
      update_frequency_hz_(update_frequency_hz),
      cycle_time_us_(static_cast<int>(1000000.0 / update_frequency_hz)),
      cycle_time_ms_(1000.0 / update_frequency_hz),
      initialized_(false),
      operational_(false),
      slave_count_(0),
      expected_wkc_(0),
      actual_wkc_(0),
      cycle_count_(0),
      has_errors_(false),
      last_cycle_time_(std::chrono::high_resolution_clock::now()) {
    
    // Initialize motor manager for automatic motor type detection
    motor_manager_ = std::make_unique<MotorManager>();
    
    std::cout << "NetworkController created for interface: " << interface_name_ 
              << " at " << update_frequency_hz_ << "Hz (" << cycle_time_ms_ << "ms cycle)" << std::endl;
    
    clearError();
}

NetworkController::~NetworkController() {
    stopOperation();
}

bool NetworkController::initialize() {
    std::cout << "Initializing EtherCAT master on " << interface_name_ << std::endl;
    
    if (initialized_.load()) {
        setError("EtherCAT master already initialized");
        return false;
    }
    
    try {
        // Initialize EtherCAT master
        if (ec_init(interface_name_.c_str())) {
            // Allocate IOmap buffer
            iomap_.resize(4096);
            std::fill(iomap_.begin(), iomap_.end(), 0);
            
            std::cout << "EtherCAT master initialized successfully on " << interface_name_ << std::endl;
            initialized_.store(true);
            clearError();
            return true;
        } else {
            setError("Failed to initialize EtherCAT master on " + interface_name_ + 
                    " (try running with sudo or check network permissions)");
            return false;
        }
    } catch (const std::exception& e) {
        setError("Exception during EtherCAT initialization: " + std::string(e.what()));
        return false;
    }
}

bool NetworkController::scanNetwork() {
    if (!initialized_.load()) {
        setError("Cannot scan network: EtherCAT master not initialized");
        return false;
    }
    
    std::cout << "Scanning EtherCAT network..." << std::endl;
    
    try {
        // Scan network for slaves
        int detected_slaves = ec_config_init(FALSE);
        slave_count_.store(detected_slaves);
        
        std::cout << "Network scan complete: found " << detected_slaves << " EtherCAT slaves" << std::endl;
        
        clearError();
        return true;
        
    } catch (const std::exception& e) {
        setError("Exception during network scan: " + std::string(e.what()));
        return false;
    }
}

bool NetworkController::configureSlaves() {
    if (!initialized_.load()) {
        setError("Cannot configure slaves: EtherCAT master not initialized");
        return false;
    }
    
    int slave_count = slave_count_.load();
    if (slave_count == 0) {
        setError("Cannot configure slaves: no slaves detected");
        return false;
    }
    
    std::cout << "Configuring " << slave_count << " EtherCAT slaves..." << std::endl;
    
    try {
        // Map process data
        int iomap_size = ec_config_map(iomap_.data());
        
        if (iomap_size == 0) {
            setError("Failed to map process data: no IOmap generated");
            return false;
        }
        
        std::cout << "Process data mapped successfully (IOmap size: " << iomap_size << " bytes)" << std::endl;
        
        // Configure distributed clocks if supported
        ec_configdc();
        
        // Initialize slave info after mapping (PDO sizes now available)
        if (slave_count_.load() > 0) {
            initializeSlaveInfo();
        }
        
        // Setup PDO pointers for all slaves
        if (!setupPDOPointers()) {
            setError("Failed to setup PDO pointers");
            return false;
        }
        
        std::cout << "All slaves configured successfully" << std::endl;
        clearError();
        return true;
        
    } catch (const std::exception& e) {
        setError("Exception during slave configuration: " + std::string(e.what()));
        return false;
    }
}

bool NetworkController::startOperation() {
    if (!initialized_.load()) {
        setError("Cannot start operation: EtherCAT master not initialized");
        return false;
    }
    
    int slave_count = slave_count_.load();
    if (slave_count == 0) {
        setError("Cannot start operation: no slaves configured");
        return false;
    }
    
    std::cout << "Starting EtherCAT network operation..." << std::endl;
    
    try {
        // Initialize output data with safe values (simplified from old working code)
        std::cout << "Initializing output data with safe values..." << std::endl;
        {
            std::lock_guard<std::mutex> lock(slaves_mutex_);
            for (auto& slave : slaves_) {
                if (slave.output_pdo != nullptr) {
                    // Initialize with CIA402 compliant control word for SOMANET
                    slave.output_pdo->controlword = 0x0006;          // CIA402 Shutdown state
                    slave.output_pdo->modes_of_operation = 0x00;      // No operation mode
                    slave.output_pdo->target_torque = 0;             // Zero torque
                    slave.output_pdo->target_position = 0;           // Zero position
                    slave.output_pdo->target_velocity = 0;           // Zero velocity  
                    slave.output_pdo->torque_offset = 0;             // Zero offset
                    slave.output_pdo->tuning_command = 0;            // No tuning
                    slave.output_pdo->physical_outputs = 0;          // All outputs off
                    slave.output_pdo->bit_mask = 0;                  // No mask
                    slave.output_pdo->user_mosi = 0;                 // User data clear
                    slave.output_pdo->velocity_offset = 0;           // Zero offset
                }
            }
        }
        
        // Request operational state (direct transition like old working code)
        std::cout << "Request operational state for all slaves" << std::endl;
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_writestate(0);
        
        int chk = 200;
        do {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
        
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
            std::cout << "Operational state reached for all slaves" << std::endl;
        } else {
            setError("Not all slaves reached operational state");
            return false;
        }
        
        // Calculate expected working counter
        expected_wkc_.store((ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC);
        
        // Set operational flag before performing communication
        operational_.store(true);
        
        // Perform initial communication cycle
        int wkc = performCommunicationCycle();
        
        if (wkc < expected_wkc_.load()) {
            std::cout << "Warning: Working counter lower than expected (got: " 
                      << wkc << ", expected: " << expected_wkc_.load() << ")" << std::endl;
        }
        std::cout << "EtherCAT network operational (WKC: " << wkc 
                  << "/" << expected_wkc_.load() << ")" << std::endl;
        
        clearError();
        return true;
        
    } catch (const std::exception& e) {
        setError("Exception during operation start: " + std::string(e.what()));
        return false;
    }
}

void NetworkController::stopOperation() {
    if (!initialized_.load()) {
        return; // Already stopped
    }
    
    std::cout << "Stopping EtherCAT network operation..." << std::endl;
    
    operational_.store(false);
    
    try {
        // Request all slaves to go to safe state
        if (slave_count_.load() > 0) {
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            
            // Brief communication to ensure state change
            auto next_cycle = std::chrono::steady_clock::now() + 
                             std::chrono::microseconds(cycle_time_us_);
            
            for (int i = 0; i < 100; i++) {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                
                std::this_thread::sleep_until(next_cycle);
                next_cycle += std::chrono::microseconds(cycle_time_us_);
            }
            
            // Final state transition to init
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
        }
        
        // Close EtherCAT master
        ec_close();
        
        // Reset state
        {
            std::lock_guard<std::mutex> lock(slaves_mutex_);
            slaves_.clear();
        }
        
        slave_count_.store(0);
        expected_wkc_.store(0);
        actual_wkc_.store(0);
        initialized_.store(false);
        
        iomap_.clear();
        
        std::cout << "EtherCAT network stopped successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during network stop: " << e.what() << std::endl;
    }
}

int NetworkController::performCommunicationCycle() {
    if (!operational_.load()) {
        return 0;
    }
    
    auto cycle_start = std::chrono::high_resolution_clock::now();
    
    // Send process data to slaves
    ec_send_processdata();
    
    // Receive process data from slaves
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Update statistics
    actual_wkc_.store(wkc);
    cycle_count_.fetch_add(1);
    
    // Check for communication errors
    if (wkc < expected_wkc_.load()) {
        has_errors_.store(true);
    }
    
    last_cycle_time_ = cycle_start;
    
    return wkc;
}

bool NetworkController::isOperational() const {
    return operational_.load();
}

OutputPDO* NetworkController::getOutputPDO(int slave_index) {
    if (!isValidSlaveIndex(slave_index)) {
        return nullptr;
    }
    
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    return slaves_[slave_index - 1].output_pdo;
}

const InputPDO* NetworkController::getInputPDO(int slave_index) const {
    if (!isValidSlaveIndex(slave_index)) {
        return nullptr;
    }
    
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    return slaves_[slave_index - 1].input_pdo;
}

NetworkStatus NetworkController::getNetworkStatus() const {
    NetworkStatus status;
    status.initialized = initialized_.load();
    status.operational = operational_.load();
    status.slave_count = slave_count_.load();
    status.expected_wkc = expected_wkc_.load();
    status.actual_wkc = actual_wkc_.load();
    status.cycle_count = cycle_count_.load();
    status.cycle_time_us = cycle_time_ms_ * 1000.0;
    
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        status.last_error = last_error_;
    }
    
    return status;
}

std::vector<SlaveInfo> NetworkController::getSlaveList() const {
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    return slaves_;
}

int NetworkController::getSlaveCount() const {
    return slave_count_.load();
}

SlaveInfo NetworkController::getSlaveInfo(int slave_index) const {
    if (!isValidSlaveIndex(slave_index)) {
        return SlaveInfo{}; // Return empty/invalid info
    }
    
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    return slaves_[slave_index - 1];
}

bool NetworkController::checkSlaveStates() {
    if (!operational_.load()) {
        return false;
    }
    
    try {
        // Check if all slaves are still in operational state
        int state_check = ec_statecheck(0, EC_STATE_OPERATIONAL, 1000); // 1ms timeout
        
        if (state_check != EC_STATE_OPERATIONAL) {
            setError("Slave state check failed (state: 0x" + std::to_string(state_check) + ")");
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        setError("Exception during slave state check: " + std::string(e.what()));
        return false;
    }
}

bool NetworkController::hasDataErrors() const {
    return has_errors_.load();
}

std::string NetworkController::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

double NetworkController::getUpdateFrequencyHz() const {
    return update_frequency_hz_;
}

int NetworkController::getCycleTimeUs() const {
    return cycle_time_us_;
}

double NetworkController::getCycleTimeMs() const {
    return cycle_time_ms_;
}

// Private helper methods

void NetworkController::setError(const std::string& error_message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error_message;
    has_errors_.store(true);
    std::cerr << "NetworkController Error: " << error_message << std::endl;
}

void NetworkController::clearError() {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_.clear();
    has_errors_.store(false);
}

void NetworkController::initializeSlaveInfo() {
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    slaves_.clear();
    
    int slave_count = slave_count_.load();
    slaves_.reserve(slave_count);
    
    for (int i = 1; i <= slave_count; i++) {
        SlaveInfo slave_info;
        slave_info.slave_index = i;
        slave_info.slave_name = std::string(ec_slave[i].name);
        slave_info.product_id = ec_slave[i].eep_id;
        slave_info.vendor_id = ec_slave[i].eep_man;
        slave_info.current_state = ec_slave[i].state;
        slave_info.output_pdo_size = ec_slave[i].Obytes;
        slave_info.input_pdo_size = ec_slave[i].Ibytes;
        slave_info.output_pdo = nullptr; // Will be set in setupPDOPointers
        slave_info.input_pdo = nullptr;  // Will be set in setupPDOPointers
        slave_info.configured = false;
        
        slaves_.push_back(std::move(slave_info));
        
        std::cout << "Slave " << i << ": " << slave_info.slave_name 
                  << " (ID: 0x" << std::hex << slave_info.product_id << std::dec
                  << ", OUT: " << slave_info.output_pdo_size 
                  << "B, IN: " << slave_info.input_pdo_size << "B)" << std::endl;
    }
}

bool NetworkController::setupPDOPointers() {
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    
    for (auto& slave : slaves_) {
        if (slave.slave_index <= slave_count_.load()) {
            // Setup output PDO pointer
            if (slave.output_pdo_size >= sizeof(OutputPDO)) {
                slave.output_pdo = reinterpret_cast<OutputPDO*>(ec_slave[slave.slave_index].outputs);
            } else {
                setError("Slave " + std::to_string(slave.slave_index) + 
                        " output PDO size too small (" + std::to_string(slave.output_pdo_size) + 
                        " bytes, need " + std::to_string(sizeof(OutputPDO)) + ")");
                return false;
            }
            
            // Setup input PDO pointer
            if (slave.input_pdo_size >= sizeof(InputPDO)) {
                slave.input_pdo = reinterpret_cast<InputPDO*>(ec_slave[slave.slave_index].inputs);
            } else {
                setError("Slave " + std::to_string(slave.slave_index) + 
                        " input PDO size too small (" + std::to_string(slave.input_pdo_size) + 
                        " bytes, need " + std::to_string(sizeof(InputPDO)) + ")");
                return false;
            }
            
            slave.configured = true;
            
            std::cout << "PDO pointers configured for slave " << slave.slave_index 
                      << " (" << slave.slave_name << ")" << std::endl;
        }
    }
    
    return true;
}

bool NetworkController::isValidSlaveIndex(int slave_index) const {
    return (slave_index >= 1 && slave_index <= slave_count_.load());
}

// === Motor Detection and Management Methods ===

DetectedMotorInfo NetworkController::getDetectedMotorInfo(int slave_index) const {
    if (!isValidSlaveIndex(slave_index) || !motor_manager_) {
        // Return empty info for invalid slave or uninitialized motor manager
        DetectedMotorInfo empty_info;
        empty_info.slave_index = slave_index;
        empty_info.product_id = 0;
        empty_info.detected_type = MotorType::UNKNOWN;
        empty_info.config_file_path = "";
        empty_info.type_name = "Invalid Slave";
        return empty_info;
    }
    
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    
    // Find slave by index
    for (const auto& slave : slaves_) {
        if (slave.slave_index == slave_index) {
            return motor_manager_->getMotorInfo(slave_index, slave.product_id);
        }
    }
    
    // Slave not found
    DetectedMotorInfo not_found_info;
    not_found_info.slave_index = slave_index;
    not_found_info.product_id = 0;
    not_found_info.detected_type = MotorType::UNKNOWN;
    not_found_info.config_file_path = "";
    not_found_info.type_name = "Slave Not Found";
    return not_found_info;
}

std::vector<DetectedMotorInfo> NetworkController::getAllDetectedMotors() const {
    std::vector<DetectedMotorInfo> detected_motors;
    
    if (!motor_manager_) {
        return detected_motors; // Return empty vector if motor manager not initialized
    }
    
    std::lock_guard<std::mutex> lock(slaves_mutex_);
    
    for (const auto& slave : slaves_) {
        DetectedMotorInfo motor_info = motor_manager_->getMotorInfo(slave.slave_index, slave.product_id);
        detected_motors.push_back(motor_info);
        
        std::cout << "Detected motor: " << motor_info.type_name 
                  << " at slave " << motor_info.slave_index 
                  << " (ID: 0x" << std::hex << motor_info.product_id << std::dec << ")" << std::endl;
    }
    
    return detected_motors;
}

const MotorManager* NetworkController::getMotorManager() const {
    return motor_manager_.get();
}

bool NetworkController::flushControlWordUpdate(int slave_index) {
    if (!operational_.load()) {
        return false;
    }
    
    if (!isValidSlaveIndex(slave_index)) {
        return false;
    }
    
    try {
        // Immediately send process data to slaves
        ec_send_processdata();
        
        // Receive process data with short timeout to verify response
        int wkc = ec_receive_processdata(1000); // 1ms timeout
        
        // Update statistics
        actual_wkc_.store(wkc);
        
        // Verify we got expected response
        if (wkc >= expected_wkc_.load()) {
            return true;
        } else {
            return false;
        }
        
    } catch (const std::exception& e) {
        return false;
    }
}

// Helper function to convert EtherCAT state to string
std::string NetworkController::getStateString(uint16_t state) const {
    switch (state) {
        case 0x01: return "INIT";
        case 0x02: return "PREOP"; 
        case 0x04: return "SAFEOP";
        case 0x08: return "OPERATIONAL";
        default: return "UNKNOWN(0x" + std::to_string(state) + ")";
    }
}

} // namespace synapticon_motor