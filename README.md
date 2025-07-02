# Draco4 Multi-Motor EtherCAT Control Framework

A high-performance C++ framework for controlling multiple servo motors via EtherCAT communication using the SOEM (Simple Open EtherCAT Master) library. Supports single and multi-network architectures for complex robotic systems.

**📖 [Complete Documentation](https://lklinus2609.github.io/draco4_motor_control/)** - Comprehensive usage guide, API reference, configuration, and safety information.

## Architecture Overview

The framework is built around a multi-layered architecture:

- **NetworkController**: Manages EtherCAT communication for a single network
- **MotorController**: Controls individual motors within a network  
- **MotorManager**: Automatic motor type detection and configuration
- **RobotController**: Coordinates multiple networks for complex robotic systems

## Features

- **Multi-Motor Support**: Control multiple motors on single or multiple EtherCAT networks
- **CIA402 State Machine**: Complete implementation for all motor types
- **Multiple Control Modes**: Velocity, position, and torque control with gear ratio support
- **Safety Features**: Torque ramping, position rate limiting, fault recovery, and emergency stop
- **Automatic Detection**: MotorManager automatically detects and configures motor types
- **Professional Documentation**: Comprehensive Doxygen-style API documentation
- **Configuration Management**: CSV-based motor configuration with SDO parameter upload

## Supported Motors

- **JD8 Series**: Fully supported with position, velocity, and torque control
- **JD10/JD12 Series**: Compatible (same PDO structure as JD8)
- **Future Motors**: Extensible architecture for new motor types

## Requirements

- CMake 3.16 or higher
- C++17 compatible compiler (GCC, Clang)
- SOEM library (Simple Open EtherCAT Master)
- Root privileges for EtherCAT network access

### Installing SOEM

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install libsoem-dev
```

**From source:**
```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
mkdir build && cd build
cmake ..
make
sudo make install
```

## Building

```bash
mkdir build
cd build
cmake ..
make
```


## Test Programs

The project includes comprehensive test programs:

- `test_jd8_position` - Position control with gear ratio scaling and 90-degree moves
- `test_jd8_velocity` - Velocity control with precision timing analysis  
- `test_jd8_torque` - Torque control with safety limits and ramping
- `test_multi_motor_simulation` - Multi-motor simulation without hardware
- `test_multi_motor_hardware` - Multi-motor hardware validation
- `test_multi_network_simulation` - Multi-network simulation testing
- `test_multi_network_hardware` - Multi-network hardware validation

Run with:
```bash
sudo ./test_jd8_velocity eth0  # Replace with your interface
sudo ./test_multi_motor_hardware eth0
sudo ./test_multi_network_hardware
```


## Safety Features

- **Torque Ramping**: Smooth torque transitions to prevent mechanical stress
- **Position Rate Limiting**: Prevents excessive position changes per cycle
- **Fault Recovery**: Automatic torque ramp-down on fault detection
- **Emergency Stop**: Immediate motion halt via `eStop()` method
- **Safety Limits**: Configurable torque and velocity limits
- **Gear Ratio Support**: Automatic scaling between motor and output shaft

## Safety Notes

- **Root Privileges**: EtherCAT communication requires root privileges
- **Network Interface**: Ensure specified interfaces are connected to EtherCAT networks
- **Real-time Timing**: Maintain configured update frequency for optimal performance
- **Motor Safety**: Always test with low torque/velocity limits initially
- **Emergency Stop**: Implement proper emergency stop procedures in your application

## Contributing

This is a motor control project focused on industrial automation. Please ensure any contributions maintain safety and real-time performance requirements.

## License

[Add your license information here]