# Draco4 Multi-Motor EtherCAT Control Framework

---
[Home](index.md) | [API Reference](api-reference.md) | [Developer Guide](developer-guide.md) | [Usage Examples](usage-examples.md)
---

A high-performance C++ framework for controlling multiple servo motors via EtherCAT communication using the SOEM (Simple Open EtherCAT Master) library. Supports single and multi-network architectures for complex robotic systems.

## Overview

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

## Installation

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

### Building the Framework

```bash
mkdir build
cd build
cmake ..
make
```


## Configuration

### Motor Configuration Files

Load motor parameters from CSV files:

```cpp
motor.loadConfig("config/JDLINK8_config_file.csv");
motor.uploadConfig();  // Upload to motor via SDO
```


## Safety

### Safety Features

- **Torque Ramping**: Smooth torque transitions to prevent mechanical stress
- **Position Rate Limiting**: Prevents excessive position changes per cycle
- **Fault Recovery**: Automatic torque ramp-down on fault detection
- **Emergency Stop**: Immediate motion halt via `eStop()` method
- **Safety Limits**: Configurable torque and velocity limits
- **Gear Ratio Support**: Automatic scaling between motor and output shaft

### Safety Requirements

- **Root Privileges**: EtherCAT communication requires root privileges
- **Network Interface**: Ensure specified interfaces are connected to EtherCAT networks
- **Real-time Timing**: Maintain configured update frequency for optimal performance
- **Motor Safety**: Always test with low torque/velocity limits initially
- **Emergency Stop**: Implement proper emergency stop procedures in your application

---

## Documentation

- **[API Reference](api-reference.md)** - Complete API documentation for all classes and methods
- **[Developer Guide](developer-guide.md)** - Comprehensive development guide and troubleshooting
- [Usage Examples](usage-examples.md) - Additional code examples