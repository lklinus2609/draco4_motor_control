/**
 * @file motor_manager.hpp
 * @brief Motor Type Detection and Configuration Management
 * 
 * This header defines the MotorManager class for automatic motor type detection
 * based on EtherCAT product IDs and configuration file management for different
 * motor types in multi-motor networks.
 */

#pragma once

#include <string>
#include <map>
#include <cstdint>

namespace synapticon_motor {

/**
 * @brief Supported motor types for automatic detection
 */
enum class MotorType {
    JD8,                ///< JD8 motor type (known product ID)
    JD10,               ///< JD10 motor type (placeholder, uses JD8 config)
    JD12,               ///< JD12 motor type (placeholder, uses JD8 config)
    UNKNOWN             ///< Unknown/unsupported motor type
};

/**
 * @brief Information about a detected motor slave
 */
struct DetectedMotorInfo {
    int slave_index;                ///< EtherCAT slave index (1-based)
    uint32_t product_id;            ///< EtherCAT product ID from slave
    MotorType detected_type;        ///< Detected motor type
    std::string config_file_path;   ///< Recommended configuration file path
    std::string type_name;          ///< Human-readable type name
};

/**
 * @brief Motor Type Detection and Configuration Management
 * 
 * Provides automatic motor type detection based on EtherCAT product IDs
 * and manages configuration file paths for different motor types.
 * Supports current JD8 motors and placeholder support for JD10/JD12.
 */
class MotorManager {
public:
    /**
     * @brief Constructor - initializes motor type mappings
     */
    MotorManager();
    
    /**
     * @brief Destructor
     */
    ~MotorManager() = default;
    
    // === Motor Type Detection ===
    
    /**
     * @brief Detect motor type from EtherCAT product ID
     * @param product_id EtherCAT product ID from slave device
     * @return Detected MotorType (UNKNOWN if not recognized)
     */
    MotorType detectMotorType(uint32_t product_id) const;
    
    /**
     * @brief Get motor information for detected slave
     * @param slave_index EtherCAT slave index
     * @param product_id EtherCAT product ID from slave
     * @return DetectedMotorInfo structure with all motor details
     */
    DetectedMotorInfo getMotorInfo(int slave_index, uint32_t product_id) const;
    
    // === Configuration Management ===
    
    /**
     * @brief Get configuration file path for motor type
     * @param type Motor type
     * @return Path to configuration file, empty string if not found
     */
    std::string getConfigFilePath(MotorType type) const;
    
    /**
     * @brief Get human-readable name for motor type
     * @param type Motor type
     * @return Type name string
     */
    std::string getTypeName(MotorType type) const;
    
    // === Motor Type Registration ===
    
    /**
     * @brief Register a new motor type with product ID mapping
     * @param product_id EtherCAT product ID
     * @param type Motor type to associate
     * @param config_file_path Path to configuration file
     * @param type_name Human-readable type name
     * @return true if registration successful
     */
    bool registerMotorType(uint32_t product_id, 
                          MotorType type,
                          const std::string& config_file_path,
                          const std::string& type_name);
    
    // === Information Queries ===
    
    /**
     * @brief Check if motor type is supported
     * @param type Motor type to check
     * @return true if motor type is supported
     */
    bool isSupported(MotorType type) const;
    
    /**
     * @brief Get list of all registered product IDs
     * @return Map of product_id -> motor_type for all registered motors
     */
    std::map<uint32_t, MotorType> getRegisteredProductIDs() const;
    
    /**
     * @brief Get count of registered motor types
     * @return Number of registered motor types
     */
    size_t getRegisteredTypeCount() const;

private:
    // === Internal Data Structures ===
    
    std::map<uint32_t, MotorType> product_id_map_;      ///< Product ID -> Motor Type mapping
    std::map<MotorType, std::string> config_file_map_;  ///< Motor Type -> Config file path
    std::map<MotorType, std::string> type_name_map_;    ///< Motor Type -> Human readable name
    
    // === Internal Helper Methods ===
    
    /**
     * @brief Initialize default motor type mappings
     */
    void initializeDefaultMappings();
    
    /**
     * @brief Validate motor type
     * @param type Motor type to validate
     * @return true if valid motor type
     */
    bool isValidMotorType(MotorType type) const;
};

} // namespace synapticon_motor