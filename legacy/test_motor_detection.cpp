/**
 * @file test_motor_detection.cpp
 * @brief Test motor type detection functionality
 * 
 * Tests the MotorManager class for automatic motor type detection
 * and configuration file mapping without requiring hardware.
 */

#include "motor_manager.hpp"
#include <iostream>
#include <iomanip>

using namespace synapticon_motor;

void testMotorTypeDetection() {
    std::cout << "=== Motor Type Detection Test ===" << std::endl;
    
    MotorManager motor_manager;
    
    // Test known product IDs
    uint32_t test_product_ids[] = {
        0x12345678,  // JD8 placeholder
        0x12345679,  // JD10 placeholder
        0x1234567A,  // JD12 placeholder
        0x99999999   // Unknown product ID
    };
    
    for (uint32_t product_id : test_product_ids) {
        MotorType detected_type = motor_manager.detectMotorType(product_id);
        std::string type_name = motor_manager.getTypeName(detected_type);
        std::string config_path = motor_manager.getConfigFilePath(detected_type);
        
        std::cout << "Product ID: 0x" << std::hex << product_id << std::dec
                  << " -> Type: " << type_name
                  << " -> Config: " << config_path << std::endl;
    }
}

void testMotorInfoGeneration() {
    std::cout << "\n=== Motor Info Generation Test ===" << std::endl;
    
    MotorManager motor_manager;
    
    // Simulate detected slaves
    struct TestSlave {
        int slave_index;
        uint32_t product_id;
    };
    
    TestSlave test_slaves[] = {
        {1, 0x12345678},  // JD8
        {2, 0x12345679},  // JD10
        {3, 0x1234567A},  // JD12
        {4, 0x11111111}   // Unknown
    };
    
    for (const auto& slave : test_slaves) {
        DetectedMotorInfo info = motor_manager.getMotorInfo(slave.slave_index, slave.product_id);
        
        std::cout << "Slave " << info.slave_index << ": "
                  << info.type_name << " (ID: 0x" << std::hex << info.product_id << std::dec << ")"
                  << " -> " << info.config_file_path << std::endl;
    }
}

void testMotorManagerQueries() {
    std::cout << "\n=== Motor Manager Queries Test ===" << std::endl;
    
    MotorManager motor_manager;
    
    std::cout << "Registered motor types: " << motor_manager.getRegisteredTypeCount() << std::endl;
    
    // Test support queries
    MotorType types_to_test[] = {
        MotorType::JD8,
        MotorType::JD10,
        MotorType::JD12,
        MotorType::UNKNOWN
    };
    
    for (MotorType type : types_to_test) {
        bool supported = motor_manager.isSupported(type);
        std::string type_name = motor_manager.getTypeName(type);
        std::cout << type_name << " supported: " << (supported ? "Yes" : "No") << std::endl;
    }
    
    // Display all registered product IDs
    std::cout << "\nRegistered Product IDs:" << std::endl;
    auto product_map = motor_manager.getRegisteredProductIDs();
    for (const auto& pair : product_map) {
        std::cout << "  0x" << std::hex << pair.first << std::dec 
                  << " -> " << motor_manager.getTypeName(pair.second) << std::endl;
    }
}

void testDynamicRegistration() {
    std::cout << "\n=== Dynamic Registration Test ===" << std::endl;
    
    MotorManager motor_manager;
    
    // Test registering a new motor type
    uint32_t test_product_id = 0xABCDEF00;
    bool success = motor_manager.registerMotorType(
        test_product_id,
        MotorType::JD8,  // Register as JD8 type
        "config/test_motor_config.csv",
        "Test Motor JD8"
    );
    
    std::cout << "Dynamic registration " << (success ? "successful" : "failed") << std::endl;
    
    if (success) {
        MotorType detected = motor_manager.detectMotorType(test_product_id);
        std::cout << "Test motor detection: " << motor_manager.getTypeName(detected) << std::endl;
    }
}

int main() {
    std::cout << "Motor Detection Functionality Test" << std::endl;
    std::cout << "===================================" << std::endl;
    
    try {
        testMotorTypeDetection();
        testMotorInfoGeneration();
        testMotorManagerQueries();
        testDynamicRegistration();
        
        std::cout << "\n✅ All motor detection tests completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}