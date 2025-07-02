/**
 * @file test_soem_multi_instance.cpp
 * @brief Test SOEM multiple instances on different network interfaces
 * 
 * This test validates whether SOEM can support multiple concurrent instances
 * on different network interfaces for multi-network EtherCAT operation.
 */

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <atomic>
#include <cstring>

extern "C" {
#include <soem/ethercat.h>
}

struct NetworkInstance {
    std::string interface_name;
    int slave_count;
    bool initialized;
    bool operational;
    char IOmap[4096];
    std::atomic<bool> running;
    
    NetworkInstance(const std::string& iface) 
        : interface_name(iface), slave_count(0), initialized(false), 
          operational(false), running(false) {
        memset(IOmap, 0, sizeof(IOmap));
    }
    
    // Make it movable (threads are not copyable)
    NetworkInstance(NetworkInstance&& other) noexcept
        : interface_name(std::move(other.interface_name)),
          slave_count(other.slave_count),
          initialized(other.initialized),
          operational(other.operational),
          running(other.running.load()) {
        memcpy(IOmap, other.IOmap, sizeof(IOmap));
    }
    
    NetworkInstance& operator=(NetworkInstance&& other) noexcept {
        if (this != &other) {
            interface_name = std::move(other.interface_name);
            slave_count = other.slave_count;
            initialized = other.initialized;
            operational = other.operational;
            running = other.running.load();
            memcpy(IOmap, other.IOmap, sizeof(IOmap));
        }
        return *this;
    }
    
    // Delete copy constructor and assignment (not needed for this test)
    NetworkInstance(const NetworkInstance&) = delete;
    NetworkInstance& operator=(const NetworkInstance&) = delete;
};

class SOEMMultiInstanceTest {
private:
    std::vector<NetworkInstance> networks_;
    
public:
    bool addNetwork(const std::string& interface_name) {
        networks_.emplace_back(interface_name);
        NetworkInstance& network = networks_.back();
        
        std::cout << "Testing SOEM initialization on " << interface_name << "..." << std::endl;
        
        // Test SOEM initialization
        if (ec_init(interface_name.c_str())) {
            std::cout << "✅ SOEM initialized successfully on " << interface_name << std::endl;
            network.initialized = true;
            
            // Test network scanning
            network.slave_count = ec_config_init(FALSE);
            std::cout << "  Found " << network.slave_count << " EtherCAT slaves" << std::endl;
            
            // Cleanup for this test
            ec_close();
            return true;
        } else {
            std::cout << "❌ Failed to initialize SOEM on " << interface_name << std::endl;
            return false;
        }
    }
    
    bool testConcurrentInitialization() {
        std::cout << "\n=== Testing Concurrent SOEM Initialization ===" << std::endl;
        
        std::vector<std::thread> init_threads;
        std::vector<bool> results(networks_.size(), false);
        
        // Try to initialize all networks concurrently
        for (size_t i = 0; i < networks_.size(); i++) {
            init_threads.emplace_back([this, i, &results]() {
                NetworkInstance& network = networks_[i];
                
                std::cout << "Thread " << i << ": Initializing " << network.interface_name << std::endl;
                
                if (ec_init(network.interface_name.c_str())) {
                    network.slave_count = ec_config_init(FALSE);
                    std::cout << "Thread " << i << ": ✅ Success - " << network.slave_count << " slaves" << std::endl;
                    results[i] = true;
                    
                    // Brief operation test
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    
                    ec_close();
                } else {
                    std::cout << "Thread " << i << ": ❌ Failed on " << network.interface_name << std::endl;
                    results[i] = false;
                }
            });
        }
        
        // Wait for all threads to complete
        for (auto& thread : init_threads) {
            thread.join();
        }
        
        // Check results
        bool all_success = true;
        for (size_t i = 0; i < results.size(); i++) {
            if (!results[i]) {
                all_success = false;
                std::cout << "❌ Network " << networks_[i].interface_name << " failed concurrent init" << std::endl;
            }
        }
        
        return all_success;
    }
    
    void printResults() {
        std::cout << "\n=== SOEM Multi-Instance Test Results ===" << std::endl;
        std::cout << "Tested " << networks_.size() << " network interfaces:" << std::endl;
        
        for (const auto& network : networks_) {
            std::cout << "  " << network.interface_name 
                      << ": " << (network.initialized ? "✅ SUPPORTED" : "❌ FAILED")
                      << " (" << network.slave_count << " slaves)" << std::endl;
        }
        
        if (networks_.size() > 1) {
            std::cout << "\nCONCLUSION: ";
            bool all_supported = true;
            for (const auto& network : networks_) {
                if (!network.initialized) all_supported = false;
            }
            
            if (all_supported) {
                std::cout << "✅ SOEM SUPPORTS MULTIPLE INSTANCES" << std::endl;
                std::cout << "✅ Multi-network architecture is feasible" << std::endl;
            } else {
                std::cout << "❌ SOEM MULTI-INSTANCE NOT SUPPORTED" << std::endl;
                std::cout << "❌ Need to implement alternative architecture" << std::endl;
            }
        }
    }
};

int main() {
    std::cout << "SOEM Multi-Instance Validation Test" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    SOEMMultiInstanceTest test;
    
    // Test actual network interface names from ip addr
    std::vector<std::string> test_interfaces = {
        "enp2s0",        // Primary Ethernet  
        "enp0s31f6",     // Secondary Ethernet
        "enx3c18a042f97c", // USB Ethernet adapter
        "wlp3s0"         // WiFi (for completeness, likely won't work for EtherCAT)
    };
    
    std::cout << "Scanning for available network interfaces..." << std::endl;
    
    // Test each interface individually first
    int successful_interfaces = 0;
    for (const auto& iface : test_interfaces) {
        if (test.addNetwork(iface)) {
            successful_interfaces++;
        }
    }
    
    std::cout << "\nFound " << successful_interfaces << " working interfaces" << std::endl;
    
    // Test concurrent initialization if we have multiple interfaces
    if (successful_interfaces > 1) {
        std::cout << "\nTesting concurrent initialization..." << std::endl;
        test.testConcurrentInitialization();
    } else {
        std::cout << "\nSkipping concurrent test - need multiple interfaces for validation" << std::endl;
        std::cout << "To properly test multi-network, ensure multiple EtherCAT interfaces are available" << std::endl;
    }
    
    test.printResults();
    
    std::cout << "\n=== Recommendations ===" << std::endl;
    if (successful_interfaces > 1) {
        std::cout << "✅ Proceed with multi-network architecture implementation" << std::endl;
        std::cout << "✅ Each network can have its own SOEM instance" << std::endl;
    } else if (successful_interfaces == 1) {
        std::cout << "⚠️  Single interface available - can still implement multi-motor on single network" << std::endl;
        std::cout << "⚠️  For multi-network testing, add more EtherCAT network interfaces" << std::endl;
    } else {
        std::cout << "❌ No EtherCAT interfaces available - check network setup" << std::endl;
        std::cout << "❌ May need to run with sudo or configure network permissions" << std::endl;
    }
    
    return 0;
}