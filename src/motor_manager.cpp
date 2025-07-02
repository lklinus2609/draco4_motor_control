/**
 * @file motor_manager.cpp
 * @brief Implementation of MotorManager class
 * 
 * Provides motor type detection and configuration management for multi-motor
 * EtherCAT networks with automatic motor type identification.
 */

#include "motor_manager.hpp"
#include <iostream>

namespace synapticon_motor {

MotorManager::MotorManager() {
    initializeDefaultMappings();
    std::cout << "MotorManager initialized with " << getRegisteredTypeCount() 
              << " registered motor types" << std::endl;
}

void MotorManager::initializeDefaultMappings() {
    // Clear any existing mappings
    product_id_map_.clear();
    config_file_map_.clear();
    type_name_map_.clear();
    
    // JD8 Motor (actual product ID to be determined from hardware)
    // Using placeholder for now - will be updated when real ID is obtained
    registerMotorType(0x12345678, MotorType::JD8, 
                     "config/JDLINK8_config_file.csv", "JD8 Motor");
    
    // JD10 Motor (placeholder - uses JD8 config for compatibility)
    registerMotorType(0x12345679, MotorType::JD10, 
                     "config/JDLINK8_config_file.csv", "JD10 Motor (JD8 compat)");
    
    // JD12 Motor (placeholder - uses JD8 config for compatibility)  
    registerMotorType(0x1234567A, MotorType::JD12, 
                     "config/JDLINK8_config_file.csv", "JD12 Motor (JD8 compat)");
}

MotorType MotorManager::detectMotorType(uint32_t product_id) const {
    auto it = product_id_map_.find(product_id);
    if (it != product_id_map_.end()) {
        return it->second;
    }
    
    // Log unknown product ID for debugging
    std::cout << "MotorManager: Unknown product ID detected: 0x" 
              << std::hex << product_id << std::dec << std::endl;
    
    return MotorType::UNKNOWN;
}

DetectedMotorInfo MotorManager::getMotorInfo(int slave_index, uint32_t product_id) const {
    DetectedMotorInfo info;
    info.slave_index = slave_index;
    info.product_id = product_id;
    info.detected_type = detectMotorType(product_id);
    info.config_file_path = getConfigFilePath(info.detected_type);
    info.type_name = getTypeName(info.detected_type);
    
    return info;
}

std::string MotorManager::getConfigFilePath(MotorType type) const {
    auto it = config_file_map_.find(type);
    if (it != config_file_map_.end()) {
        return it->second;
    }
    
    // Default to JD8 config for unknown types (best compatibility)
    std::cout << "MotorManager: No config found for motor type, using JD8 default" << std::endl;
    return "config/JDLINK8_config_file.csv";
}

std::string MotorManager::getTypeName(MotorType type) const {
    auto it = type_name_map_.find(type);
    if (it != type_name_map_.end()) {
        return it->second;
    }
    
    // Return descriptive name for unknown types
    return "Unknown Motor Type";
}

bool MotorManager::registerMotorType(uint32_t product_id, 
                                    MotorType type,
                                    const std::string& config_file_path,
                                    const std::string& type_name) {
    if (!isValidMotorType(type)) {
        std::cerr << "MotorManager: Invalid motor type for registration" << std::endl;
        return false;
    }
    
    if (config_file_path.empty()) {
        std::cerr << "MotorManager: Empty config file path provided" << std::endl;
        return false;
    }
    
    // Register the mappings
    product_id_map_[product_id] = type;
    config_file_map_[type] = config_file_path;
    type_name_map_[type] = type_name;
    
    std::cout << "MotorManager: Registered " << type_name 
              << " (ID: 0x" << std::hex << product_id << std::dec 
              << ", Config: " << config_file_path << ")" << std::endl;
    
    return true;
}

bool MotorManager::isSupported(MotorType type) const {
    return config_file_map_.find(type) != config_file_map_.end();
}

std::map<uint32_t, MotorType> MotorManager::getRegisteredProductIDs() const {
    return product_id_map_;
}

size_t MotorManager::getRegisteredTypeCount() const {
    return config_file_map_.size();
}

bool MotorManager::isValidMotorType(MotorType type) const {
    return type == MotorType::JD8 || 
           type == MotorType::JD10 || 
           type == MotorType::JD12;
    // Note: UNKNOWN is not a valid type for registration
}

} // namespace synapticon_motor