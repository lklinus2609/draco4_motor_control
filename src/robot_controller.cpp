/**
 * @file robot_controller.cpp
 * @brief Implementation of RobotController class
 * 
 * Provides multi-network coordination for complex robotic systems.
 * Manages multiple EtherCAT networks with synchronized timing and
 * coordinated control across distributed motor systems.
 */

#include "robot_controller.hpp"
#include <iostream>
#include <algorithm>
#include <future>
#include <chrono>
#include <numeric>

namespace synapticon_motor {

RobotController::RobotController(double global_frequency_hz)
    : global_frequency_hz_(global_frequency_hz),
      global_cycle_time_us_(static_cast<int>(1000000.0 / global_frequency_hz)),
      global_cycle_count_(0),
      initialized_(false),
      operational_(false),
      emergency_stop_(false),
      shutdown_requested_(false),
      last_update_time_(std::chrono::high_resolution_clock::now()) {
    
    clearError();
    
    std::cout << "RobotController created with global frequency: " 
              << global_frequency_hz_ << "Hz (" << global_cycle_time_us_ << "μs cycle)" << std::endl;
}

RobotController::~RobotController() {
    shutdownAll();
    std::cout << "RobotController destroyed" << std::endl;
}

// === Network Management ===

bool RobotController::addNetwork(const NetworkConfig& config) {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    // Validate configuration
    if (!validateNetworkConfig(config)) {
        setError("Invalid network configuration: " + config.network_name);
        return false;
    }
    
    // Check for unique network name
    if (!isNetworkNameUnique(config.network_name)) {
        setError("Network name already exists: " + config.network_name);
        return false;
    }
    
    // Check for unique interface name
    for (const auto& [name, existing_config] : network_configs_) {
        if (existing_config.interface_name == config.interface_name) {
            setError("Interface already in use: " + config.interface_name);
            return false;
        }
    }
    
    try {
        // Create NetworkController instance
        auto network = std::make_unique<NetworkController>(config.interface_name, config.update_frequency_hz);
        
        // Store network and configuration
        networks_[config.network_name] = std::move(network);
        network_configs_[config.network_name] = config;
        
        std::cout << "Network added: " << config.network_name 
                  << " (interface: " << config.interface_name 
                  << ", frequency: " << config.update_frequency_hz << "Hz)" << std::endl;
        
        clearError();
        return true;
        
    } catch (const std::exception& e) {
        setError("Failed to create network '" + config.network_name + "': " + e.what());
        return false;
    }
}

bool RobotController::removeNetwork(const std::string& network_name) {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    auto network_it = networks_.find(network_name);
    if (network_it == networks_.end()) {
        setError("Network not found: " + network_name);
        return false;
    }
    
    // Stop network operation before removal
    network_it->second->stopOperation();
    
    // Remove from both containers
    networks_.erase(network_it);
    network_configs_.erase(network_name);
    
    std::cout << "Network removed: " << network_name << std::endl;
    clearError();
    return true;
}

NetworkController* RobotController::getNetwork(const std::string& network_name) {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    auto it = networks_.find(network_name);
    return (it != networks_.end()) ? it->second.get() : nullptr;
}

std::vector<std::string> RobotController::getNetworkNames() const {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    std::vector<std::string> names;
    names.reserve(networks_.size());
    
    for (const auto& [name, network] : networks_) {
        names.push_back(name);
    }
    
    return names;
}

size_t RobotController::getNetworkCount() const {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    return networks_.size();
}

// === Robot-Level Operations ===

bool RobotController::initializeAllNetworks() {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    if (networks_.empty()) {
        setError("No networks configured for initialization");
        return false;
    }
    
    std::cout << "Initializing " << networks_.size() << " networks..." << std::endl;
    
    // Initialize networks in parallel
    std::vector<std::future<std::pair<std::string, bool>>> initialization_futures;
    
    for (const auto& [name, network] : networks_) {
        if (!network_configs_.at(name).enabled) {
            std::cout << "Skipping disabled network: " << name << std::endl;
            continue;
        }
        
        initialization_futures.push_back(
            std::async(std::launch::async, [&name, &network]() -> std::pair<std::string, bool> {
                std::cout << "Initializing network: " << name << std::endl;
                bool success = network->initialize();
                std::cout << "Network " << name << " initialization: " 
                          << (success ? "SUCCESS" : "FAILED") << std::endl;
                return {name, success};
            })
        );
    }
    
    // Wait for all initializations to complete
    bool all_successful = true;
    std::vector<std::string> failed_networks;
    
    for (auto& future : initialization_futures) {
        auto [network_name, success] = future.get();
        if (!success) {
            all_successful = false;
            failed_networks.push_back(network_name);
        }
    }
    
    if (all_successful) {
        initialized_.store(true);
        std::cout << "✅ All networks initialized successfully" << std::endl;
        clearError();
    } else {
        std::string error_msg = "Failed to initialize networks: ";
        for (const auto& name : failed_networks) {
            error_msg += name + " ";
        }
        setError(error_msg);
        std::cout << "❌ " << error_msg << std::endl;
    }
    
    return all_successful;
}

bool RobotController::startAllNetworks() {
    if (!initialized_.load()) {
        setError("Networks must be initialized before starting");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    std::cout << "Starting operation on all networks..." << std::endl;
    
    // Scan networks first
    std::vector<std::future<std::pair<std::string, bool>>> scan_futures;
    
    for (const auto& [name, network] : networks_) {
        if (!network_configs_.at(name).enabled) continue;
        
        scan_futures.push_back(
            std::async(std::launch::async, [&name, &network]() -> std::pair<std::string, bool> {
                std::cout << "Scanning network: " << name << std::endl;
                bool success = network->scanNetwork() && network->configureSlaves();
                std::cout << "Network " << name << " scan/configure: " 
                          << (success ? "SUCCESS" : "FAILED") << std::endl;
                return {name, success};
            })
        );
    }
    
    // Wait for all scans
    bool all_scanned = true;
    for (auto& future : scan_futures) {
        auto [network_name, success] = future.get();
        if (!success) {
            all_scanned = false;
        }
    }
    
    if (!all_scanned) {
        setError("Failed to scan/configure some networks");
        return false;
    }
    
    // Start operation on all networks
    std::vector<std::future<std::pair<std::string, bool>>> start_futures;
    
    for (const auto& [name, network] : networks_) {
        if (!network_configs_.at(name).enabled) continue;
        
        start_futures.push_back(
            std::async(std::launch::async, [&name, &network]() -> std::pair<std::string, bool> {
                std::cout << "Starting operation: " << name << std::endl;
                bool success = network->startOperation();
                std::cout << "Network " << name << " start operation: " 
                          << (success ? "SUCCESS" : "FAILED") << std::endl;
                return {name, success};
            })
        );
    }
    
    // Wait for all starts
    bool all_started = true;
    std::vector<std::string> failed_networks;
    
    for (auto& future : start_futures) {
        auto [network_name, success] = future.get();
        if (!success) {
            all_started = false;
            failed_networks.push_back(network_name);
        }
    }
    
    if (all_started) {
        operational_.store(true);
        std::cout << "✅ All networks operational" << std::endl;
        clearError();
    } else {
        std::string error_msg = "Failed to start networks: ";
        for (const auto& name : failed_networks) {
            error_msg += name + " ";
        }
        setError(error_msg);
        std::cout << "❌ " << error_msg << std::endl;
    }
    
    return all_started;
}

void RobotController::updateAllNetworks() {
    if (!operational_.load() || emergency_stop_.load()) {
        return;
    }
    
    auto update_start = std::chrono::high_resolution_clock::now();
    
    // Update all networks in parallel
    std::vector<std::future<bool>> update_futures;
    
    {
        std::lock_guard<std::mutex> lock(networks_mutex_);
        
        for (const auto& [name, network] : networks_) {
            if (!network_configs_.at(name).enabled) continue;
            
            update_futures.push_back(
                std::async(std::launch::async, [this, &name]() -> bool {
                    return updateNetworkParallel(name);
                })
            );
        }
    }
    
    // Wait for all updates to complete
    for (auto& future : update_futures) {
        future.get();  // Wait for completion, errors logged internally
    }
    
    // Update timing statistics
    updateTimingStatistics();
    
    // Increment cycle count
    global_cycle_count_++;
    
    // Maintain global cycle timing
    auto update_end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        update_end - update_start);
    auto remaining = std::chrono::microseconds(global_cycle_time_us_) - elapsed;
    
    if (remaining > std::chrono::microseconds(0)) {
        std::this_thread::sleep_for(remaining);
    } else if (elapsed.count() > global_cycle_time_us_ * 1.1) {
        // Log timing violations (only if significantly over)
        std::cout << "⚠️  Global cycle time exceeded: " << elapsed.count() 
                  << "μs (target: " << global_cycle_time_us_ << "μs)" << std::endl;
    }
    
    last_update_time_ = std::chrono::high_resolution_clock::now();
}

void RobotController::emergencyStopAll() {
    emergency_stop_.store(true);
    
    std::cout << "🚨 EMERGENCY STOP ACTIVATED" << std::endl;
    
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    // Emergency stop all networks in parallel
    std::vector<std::future<void>> stop_futures;
    
    for (const auto& [name, network] : networks_) {
        stop_futures.push_back(
            std::async(std::launch::async, [&name, &network]() {
                network->stopOperation();
                std::cout << "Network " << name << " emergency stopped" << std::endl;
            })
        );
    }
    
    // Wait for all emergency stops
    for (auto& future : stop_futures) {
        future.get();
    }
    
    operational_.store(false);
    std::cout << "🚨 All networks emergency stopped" << std::endl;
}

void RobotController::shutdownAll() {
    if (shutdown_requested_.load()) return;
    
    shutdown_requested_.store(true);
    
    std::cout << "Shutting down RobotController..." << std::endl;
    
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    // Shutdown all networks
    for (const auto& [name, network] : networks_) {
        network->stopOperation();
        std::cout << "Network " << name << " shutdown" << std::endl;
    }
    
    operational_.store(false);
    initialized_.store(false);
    
    std::cout << "RobotController shutdown complete" << std::endl;
}

// === Motor Access Across Networks ===

MotorController* RobotController::getMotor(const std::string& network_name, int slave_index) {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    auto network_it = networks_.find(network_name);
    if (network_it == networks_.end()) {
        return nullptr;
    }
    
    // For now, return nullptr - motor access will be implemented when 
    // NetworkController provides motor access methods
    // TODO: Implement when NetworkController exposes motor instances
    return nullptr;
}

std::vector<MotorController*> RobotController::getAllMotors() {
    std::vector<MotorController*> all_motors;
    
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    // TODO: Implement when NetworkController exposes motor instances
    // For now, return empty vector
    
    return all_motors;
}

std::vector<MotorController*> RobotController::getNetworkMotors(const std::string& network_name) {
    std::vector<MotorController*> network_motors;
    
    // TODO: Implement when NetworkController exposes motor instances
    
    return network_motors;
}

// === Status and Diagnostics ===

bool RobotController::allNetworksOperational() const {
    if (!operational_.load()) return false;
    
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    for (const auto& [name, network] : networks_) {
        if (!network_configs_.at(name).enabled) continue;
        
        if (!network->isOperational()) {
            return false;
        }
    }
    
    return true;
}

RobotStatus RobotController::getRobotStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    RobotStatus status;
    status.initialized = initialized_.load();
    status.operational = operational_.load();
    status.total_networks = networks_.size();
    status.emergency_stop_active = emergency_stop_.load();
    status.global_cycle_time_us = global_cycle_time_us_;
    status.global_cycle_count = global_cycle_count_.load();
    status.last_error = last_error_;
    
    // Count active networks
    status.active_networks = 0;
    for (const auto& [name, network] : networks_) {
        if (network_configs_.at(name).enabled && network->isOperational()) {
            status.active_networks++;
        }
    }
    
    // Count total motors (TODO: implement when motor access available)
    status.total_motors = 0;
    
    return status;
}

std::map<std::string, NetworkStatus> RobotController::getAllNetworkStatus() const {
    std::lock_guard<std::mutex> lock(networks_mutex_);
    
    std::map<std::string, NetworkStatus> all_status;
    
    for (const auto& [name, network] : networks_) {
        all_status[name] = network->getNetworkStatus();
    }
    
    return all_status;
}

bool RobotController::isEmergencyStopActive() const {
    return emergency_stop_.load();
}

// === Timing and Performance ===

double RobotController::getGlobalFrequency() const {
    return global_frequency_hz_;
}

int RobotController::getGlobalCycleTimeUs() const {
    return global_cycle_time_us_;
}

uint64_t RobotController::getGlobalCycleCount() const {
    return global_cycle_count_.load();
}

// === Internal Helper Methods ===

bool RobotController::validateNetworkConfig(const NetworkConfig& config) const {
    if (config.network_name.empty()) {
        std::cerr << "Network name cannot be empty" << std::endl;
        return false;
    }
    
    if (config.interface_name.empty()) {
        std::cerr << "Interface name cannot be empty" << std::endl;
        return false;
    }
    
    if (config.update_frequency_hz <= 0 || config.update_frequency_hz > 10000) {
        std::cerr << "Invalid update frequency: " << config.update_frequency_hz << "Hz" << std::endl;
        return false;
    }
    
    return true;
}

bool RobotController::isNetworkNameUnique(const std::string& network_name) const {
    return networks_.find(network_name) == networks_.end();
}

void RobotController::setError(const std::string& error_message) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_error_ = error_message;
    std::cerr << "RobotController Error: " << error_message << std::endl;
}

void RobotController::clearError() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_error_.clear();
}

bool RobotController::updateNetworkParallel(const std::string& network_name) {
    NetworkController* network = nullptr;
    
    {
        std::lock_guard<std::mutex> lock(networks_mutex_);
        auto it = networks_.find(network_name);
        if (it != networks_.end()) {
            network = it->second.get();
        }
    }
    
    if (!network) {
        return false;
    }
    
    try {
        int wkc = network->performCommunicationCycle();
        return wkc > 0;  // Simple success check
    } catch (const std::exception& e) {
        std::cerr << "Network update failed for " << network_name << ": " << e.what() << std::endl;
        return false;
    }
}

void RobotController::updateTimingStatistics() {
    // TODO: Implement timing statistics collection
    // - Track cycle time variations
    // - Monitor network synchronization
    // - Detect timing violations
}

} // namespace synapticon_motor