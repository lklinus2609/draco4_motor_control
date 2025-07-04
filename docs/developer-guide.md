# Developer Guide

---
[Home](index.md) | [API Reference](api-reference.md) | [Developer Guide](developer-guide.md) | [Code Analysis](code-analysis.md) | [Usage Examples](usage-examples.md)
---

## Project Overview

Draco4 is a high-performance, real-time EtherCAT motor control system designed for complex robotics applications. This guide will help you understand and use the system to build a ROS2 SDK for humanoid robot control.

## System Architecture

### Hierarchical Design (3 Layers)

```
RobotController (Robot-wide coordination)
    ├── NetworkController (per EtherCAT interface) 
    │   ├── MotorController (individual motors)
    │   ├── MotorController 
    │   └── MotorController
    └── NetworkController (another interface)
        ├── MotorController
        └── MotorController
```

**Why this architecture?**
- **Scalability**: Add networks as your robot grows
- **Isolation**: Network failures don't affect other limbs
- **Performance**: Parallel processing across networks
- **Real-time**: 250Hz default control loops

## Real-Time Operation

### Timing Requirements

**Critical**: The system requires precise timing for stable control.

```cpp
// Main control loop - MUST run at configured frequency
auto next_cycle = std::chrono::steady_clock::now();
const auto cycle_time = std::chrono::microseconds(4000); // 250Hz = 4ms

while (operational) {
    robot->updateAllNetworks();  // EtherCAT communication
    
    // Your control logic here
    processROS2Messages();
    updateMotorCommands();
    publishFeedback();
    
    // Sleep until next cycle
    next_cycle += cycle_time;
    std::this_thread::sleep_until(next_cycle);
}
```

### Performance Tips

1. **Pre-allocate**: Avoid memory allocation in control loop
2. **Batch operations**: Update all motors together
3. **Monitor timing**: Log if cycles exceed target time
4. **Use RT kernel**: For production humanoid systems


## Data Structures

### PDO (Process Data Objects)

**OutputPDO** (Commands to motor):
```cpp
struct OutputPDO {
    uint16_t controlword;      // CIA402 control
    uint8_t modes_of_operation; // Control mode
    uint16_t target_torque;    // Torque command
    uint32_t target_position;  // Position command  
    uint32_t target_velocity;  // Velocity command
    // ... additional fields
};
```

**InputPDO** (Feedback from motor):
```cpp
struct InputPDO {
    uint16_t statusword;       // CIA402 status
    uint32_t position_actual;  // Current position
    uint32_t velocity_actual;  // Current velocity
    uint16_t torque_actual;    // Current torque
    // ... additional fields
};
```

### Network Status

```cpp
struct NetworkStatus {
    bool initialized;          // EtherCAT master ready
    bool operational;          // Network in OP state
    int slave_count;           // Number of motors found
    int expected_wkc;          // Expected working counter
    int actual_wkc;            // Actual working counter
    uint64_t cycle_count;      // Total cycles completed
    std::string last_error;    // Error message
};
```

## Performance Monitoring

```cpp
// Monitor cycle timing
auto start = std::chrono::high_resolution_clock::now();
robot->updateAllNetworks();
auto end = std::chrono::high_resolution_clock::now();

auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
if (duration.count() > 3000) {  // Warn if > 3ms (75% of 4ms cycle)
    std::cout << "Cycle time exceeded: " << duration.count() << "μs" << std::endl;
}
```

## Hardware Setup

### Requirements

**For Development**:
- Ubuntu 20.04+ with RT kernel (recommended)
- EtherCAT interface cards (e.g., Beckhoff CX series)
- Synapticon JD8/JD10/JD12 motors
- EtherCAT cables and topology

**Network Setup**:
```bash
# Configure EtherCAT interface
sudo ip link set eth0 up
sudo ethtool -s eth0 speed 100 duplex full autoneg off

# Check EtherCAT slaves
sudo ethercat slaves
```

## Troubleshooting

### Common Issues

**1. "Failed to initialize EtherCAT master"**
- Check network interface exists: `ip link show eth0`
- Verify permissions: run with `sudo` or add user to `netdev` group
- Ensure no other EtherCAT master is running

**2. "No slaves detected"**  
- Check physical connections
- Verify motor power supply
- Use `ethercat slaves` to debug

**3. "Working counter mismatch"**
- Communication errors on EtherCAT network
- Check cable quality and connections
- Verify EMI shielding

**4. Motor not enabling**
- Check motor configuration file path
- Verify SDO upload completed successfully
- Check motor statusword for specific fault codes

### Debug Output

Enable detailed logging:
```cpp
// In motor_controller.hpp, change line 16:
#define MOTOR_DEBUG_LOGGING 1
```

## Building Your ROS2 SDK

### Recommended Architecture

1. **Node Structure**:
   - Main humanoid controller node
   - Safety monitor node  
   - Diagnostic publisher node

2. **Topics**:
   - `/joint_states` - Current robot state
   - `/joint_trajectory` - Motion commands
   - `/emergency_stop` - Safety trigger
   - `/diagnostics` - System health

3. **Services**:
   - `/enable_motors` - Enable/disable motors
   - `/home_robot` - Move to home position
   - `/clear_faults` - Reset motor faults

4. **Parameters**:
   - Joint names and motor mappings
   - Safety limits per joint
   - Control gains

### Next Steps

1. **Start Simple**: Single motor, single network
2. **Add Complexity**: Multiple motors on one network  
3. **Scale Up**: Multiple networks for full humanoid
4. **Optimize**: Real-time performance tuning
5. **Integrate**: Full ROS2 ecosystem integration

The Draco4 system provides a robust foundation for your humanoid robot control. Focus on understanding the hierarchical architecture and building your ROS2 layer incrementally on top of the existing real-time motor control capabilities.