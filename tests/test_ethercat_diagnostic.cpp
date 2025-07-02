/**
 * @file test_ethercat_diagnostic.cpp
 * @brief EtherCAT diagnostic test to identify PDO configuration issues
 * 
 * This test directly uses SOEM library calls to diagnose EtherCAT slave
 * configuration and compare with our NetworkController implementation.
 */

#include "network_controller.hpp"
#include <iostream>
#include <iomanip>
#include <signal.h>

extern "C" {
#include <soem/ethercat.h>
}

volatile bool running = true;
void signal_handler(int) { running = false; }

void print_slave_info_direct() {
    std::cout << "\n=== DIRECT SOEM SLAVE INSPECTION ===" << std::endl;
    
    for (int i = 1; i <= ec_slavecount; i++) {
        std::cout << "Slave " << i << ":" << std::endl;
        std::cout << "  Name: " << ec_slave[i].name << std::endl;
        std::cout << "  Product ID: 0x" << std::hex << ec_slave[i].eep_id << std::dec << std::endl;
        std::cout << "  Output bytes: " << ec_slave[i].Obytes << std::endl;
        std::cout << "  Input bytes: " << ec_slave[i].Ibytes << std::endl;
        std::cout << "  Output bits: " << ec_slave[i].Obits << std::endl;
        std::cout << "  Input bits: " << ec_slave[i].Ibits << std::endl;
        std::cout << "  State: " << (int)ec_slave[i].state << std::endl;
        std::cout << "  AL Status: 0x" << std::hex << ec_slave[i].ALstatuscode << std::dec << std::endl;
        std::cout << "  Has DC: " << (ec_slave[i].hasdc ? "Yes" : "No") << std::endl;
        
        // Print sync manager info
        for (int sm = 0; sm < EC_MAXSM; sm++) {
            if (ec_slave[i].SM[sm].SMlength > 0) {
                std::cout << "  SM" << sm << ": Addr=0x" << std::hex << ec_slave[i].SM[sm].StartAddr 
                         << " Len=" << std::dec << ec_slave[i].SM[sm].SMlength 
                         << " Flags=0x" << std::hex << ec_slave[i].SM[sm].SMflags << std::dec << std::endl;
            }
        }
        std::cout << std::endl;
    }
}

void compare_with_network_controller(synapticon_motor::NetworkController& network) {
    std::cout << "\n=== COMPARING WITH NETWORK CONTROLLER ===" << std::endl;
    
    // Get slave info from NetworkController
    if (network.getSlaveCount() > 0) {
        auto slave_info = network.getSlaveInfo(1);
        std::cout << "NetworkController reports:" << std::endl;
        std::cout << "  Slave 1 name: " << slave_info.slave_name << std::endl;
        std::cout << "  Product ID: 0x" << std::hex << slave_info.product_id << std::dec << std::endl;
        std::cout << "  Output PDO size: " << slave_info.output_pdo_size << " bytes" << std::endl;
        std::cout << "  Input PDO size: " << slave_info.input_pdo_size << " bytes" << std::endl;
        std::cout << "  Expected Output PDO: " << sizeof(synapticon_motor::OutputPDO) << " bytes" << std::endl;
        std::cout << "  Expected Input PDO: " << sizeof(synapticon_motor::InputPDO) << " bytes" << std::endl;
        
        // Compare values
        std::cout << "\nComparison:" << std::endl;
        if (slave_info.output_pdo_size == sizeof(synapticon_motor::OutputPDO)) {
            std::cout << "  ✓ Output PDO size matches" << std::endl;
        } else {
            std::cout << "  ✗ Output PDO size mismatch! Expected " 
                     << sizeof(synapticon_motor::OutputPDO) << ", got " 
                     << slave_info.output_pdo_size << std::endl;
        }
        
        if (slave_info.input_pdo_size == sizeof(synapticon_motor::InputPDO)) {
            std::cout << "  ✓ Input PDO size matches" << std::endl;
        } else {
            std::cout << "  ✗ Input PDO size mismatch! Expected " 
                     << sizeof(synapticon_motor::InputPDO) << ", got " 
                     << slave_info.input_pdo_size << std::endl;
        }
    } else {
        std::cout << "NetworkController reports no slaves found!" << std::endl;
    }
}

void test_configuration_sequence(const std::string& interface_name) {
    std::cout << "\n=== TESTING CONFIGURATION SEQUENCE ===" << std::endl;
    
    // Step 1: Direct SOEM initialization
    std::cout << "Step 1: Direct SOEM initialization..." << std::endl;
    if (ec_init(interface_name.c_str())) {
        std::cout << "  ✓ ec_init() successful" << std::endl;
    } else {
        std::cout << "  ✗ ec_init() failed" << std::endl;
        return;
    }
    
    // Step 2: Network scan
    std::cout << "Step 2: Network scan..." << std::endl;
    int slave_count = ec_config_init(FALSE);
    std::cout << "  Found " << slave_count << " slaves" << std::endl;
    
    if (slave_count > 0) {
        print_slave_info_direct();
    }
    
    // Step 3: Test NetworkController on same interface
    std::cout << "Step 3: Testing NetworkController..." << std::endl;
    auto network = std::make_shared<synapticon_motor::NetworkController>(interface_name, 250.0);
    
    std::cout << "  NetworkController initialize()..." << std::endl;
    if (network->initialize()) {
        std::cout << "    ✓ Initialize successful" << std::endl;
    } else {
        std::cout << "    ✗ Initialize failed: " << network->getLastError() << std::endl;
        return;
    }
    
    std::cout << "  NetworkController scanNetwork()..." << std::endl;
    if (network->scanNetwork()) {
        std::cout << "    ✓ Scan successful" << std::endl;
    } else {
        std::cout << "    ✗ Scan failed: " << network->getLastError() << std::endl;
        return;
    }
    
    std::cout << "  NetworkController configureSlaves()..." << std::endl;
    if (network->configureSlaves()) {
        std::cout << "    ✓ Configure successful" << std::endl;
    } else {
        std::cout << "    ✗ Configure failed: " << network->getLastError() << std::endl;
    }
    
    compare_with_network_controller(*network);
    
    // Cleanup
    network->stopOperation();
    ec_close();
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    std::string interface_name = argv[1];
    
    std::cout << "=== ETHERCAT DIAGNOSTIC TEST ===" << std::endl;
    std::cout << "Interface: " << interface_name << std::endl;
    std::cout << "Expected OutputPDO size: " << sizeof(synapticon_motor::OutputPDO) << " bytes" << std::endl;
    std::cout << "Expected InputPDO size: " << sizeof(synapticon_motor::InputPDO) << " bytes" << std::endl;
    
    try {
        test_configuration_sequence(interface_name);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}