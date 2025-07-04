# Draco4 Code Analysis - Complete Function-Level Documentation

## Overview

The Draco4 Multi-Motor EtherCAT Control Framework is a sophisticated real-time control system designed for complex robotic applications. The architecture provides layered abstraction from individual motor control up to multi-network robot coordination, with comprehensive safety systems, automatic motor detection, and configuration-driven parameter management.

## System Architecture

The framework implements a hierarchical multi-layer control architecture:

### Layer 1: Robot Controller (`robot_controller.cpp`)
- Top-level coordinator managing multiple EtherCAT networks
- Handles network-level timing synchronization and parallel operation
- Provides system-wide emergency stop and fault management
- Coordinates initialization and shutdown across entire robot system

### Layer 2: Network Controller (`network_controller.cpp`)
- Manages individual EtherCAT network segments
- Handles real-time PDO (Process Data Object) communication
- Provides slave management and network status monitoring
- Integrates motor detection and automatic configuration

### Layer 3: Motor Controller (`motor_controller.cpp`)
- Individual motor control with CIA402 state machine implementation
- Real-time control modes: position, velocity, and torque control
- Safety systems with fault detection and recovery
- Configuration management and parameter upload

### Layer 4: Supporting Systems
- **Motor Manager**: Automatic motor type detection and configuration selection
- **SDO Manager**: Parameter upload and verification via EtherCAT SDO
- **Configuration Parser**: CSV-based motor parameter loading
- **Motor Constants**: Safety limits and conversion factors

## File-by-File Function Analysis

### 1. motor_constants.cpp
Provides configuration-driven motor constants and parameters with fallback defaults.

**Key Functions:**
- `getCountsPerRev()` (line 15): Returns encoder resolution from config or default 524288 counts/rev
- `getGearReductionRatio()` (line 22): Returns gear ratio from config or default 7.75:1
- `getRatedTorqueMNm()` (line 29): Returns rated torque from config or default 6000 mNm (6 Nm)
- `getMaxTorqueMNm()` (line 36): Returns maximum torque limit from config or default 3000 mNm
- `getMinTorqueMNm()` (line 43): Returns minimum torque (negative of max) for bidirectional control
- `getTorqueRampRate()` (line 50): Returns torque ramping rate from config or default 200 mNm/cycle
- `getMaxPositionChangePerCycle()` (line 57): Calculates maximum safe position change per cycle (4 revolutions worth of encoder counts)

**Purpose**: Acts as a safety layer providing fallback constants when configuration files aren't available while allowing motor-specific tuning when configs are loaded.

### 2. motor_configuration.cpp
Implements comprehensive CSV configuration file parsing for motor parameters.

**Core Classes:**
- `MotorConfigParser::Parameter` (line 23): Parses and stores individual parameter values with automatic type detection (hex, decimal, float, string)
- `MotorConfigParser` (line 82): Main configuration parser class

**Key Functions:**
- `parseCSV()` (line 88): Main entry point that reads and parses entire CSV configuration file
- `parseLine()` (line 139): Parses individual CSV lines in format: INDEX, SUBINDEX, VALUE
- `validateParameters()` (line 173): Validates critical parameters like encoder resolution
- `getControlGains()` (line 216): Extracts PID gains for position, velocity, and current control loops
- `getMotorSpecs()` (line 255): Extracts motor hardware specifications (encoder resolution, max speed, rated current)
- `getSafetyLimits()` (line 281): Extracts position and torque safety limits
- `calcVelScale()` (line 309): Calculates velocity scaling factor based on encoder and velocity resolution
- `getGearReductionRatio()` (line 321): Calculates gear ratio from motor/shaft revolution parameters
- `getRatedTorqueMNm()` (line 346): Converts internal torque units to millinewton-meters
- `printSummary()` (line 420): Displays comprehensive configuration summary

**Purpose**: Enables motor-agnostic operation by loading motor-specific parameters from CSV files, supporting different motor models (JD8, JD10, JD12) with their unique characteristics.

### 3. motor_controller.cpp
Main motor control class implementing EtherCAT-based control for individual motors.

**Constructor & Lifecycle:**
- `MotorController()` (line 19): Initializes motor controller with network reference, slave index, and update frequency
- `~MotorController()` (line 43): Cleanup that stops motor safely

**Core Control Functions:**
- `update()` (line 50): Main control loop called every cycle - handles torque ramping, position/velocity updates, and fault recovery
- `enable_motor()` (line 68): Implements CIA402 state machine to transition motor from shutdown to operational state
- `disable_motor()` (line 100): Safely shuts down motor and clears all commands

**Control Mode Functions:**
- `set_velocity_rpm()` (line 120): Sets target velocity with safety checks and mode switching
- `set_position_counts()` (line 158): Sets target position with gear ratio compensation (user commands output shaft, PDO controls motor shaft)
- `set_torque_millinm()` (line 199): Sets target torque with validation and mode switching

**Feedback Functions:**
- `getMotorRPM()` (line 257): Returns motor shaft RPM with gear ratio scaling to output shaft
- `getOutputRPM()` (line 284): Returns output shaft RPM directly
- `getPosition()` (line 298): Returns motor shaft position in encoder counts
- `getOutputPos()` (line 308): Returns output shaft position with gear ratio compensation
- `getTorque()` (line 320): Returns actual torque in millinewton-meters

**State Management:**
- `getStateStr()` (line 236): Returns human-readable CIA402 state string
- `get_motor_state()` (line 364): Returns structured motor state enum
- `has_fault()` (line 344): Checks for motor fault conditions
- `clear_faults()` (line 352): Resets motor faults
- `eStop()` (line 393): Emergency stop with immediate torque ramp to zero

**Internal Control Logic:**
- `enable_motor_sequence()` (line 412): Implements CIA402 state machine transitions (shutdown→ready→switched on→operational)
- `ramp_torque_command()` (line 501): Implements safe torque ramping to prevent mechanical shock
- `update_position_command()` (line 534): Handles position control with maximum step limiting
- `update_velocity_command()` (line 558): Handles velocity control with gear ratio scaling
- `handle_fault_recovery()` (line 594): Initiates fault recovery by ramping torque to zero

**Configuration Management:**
- `loadConfig()` (line 603): Loads motor configuration from CSV file
- `uploadConfig()` (line 642): Uploads complete configuration to motor via SDO
- `uploadCriticalParam()` (line 681): Uploads only critical parameters for faster startup

**Purpose**: Implements the complete control stack for a single motor, handling real-time control, safety systems, and motor-specific configuration.

### 4. motor_manager.cpp
Provides motor type detection and configuration management.

**Core Functions:**
- `MotorManager()` (line 14): Initializes with default motor type mappings
- `initializeDefaultMappings()` (line 20): Sets up product ID to motor type mappings for JD8, JD10, JD12
- `detectMotorType()` (line 40): Identifies motor type from EtherCAT product ID
- `getMotorInfo()` (line 53): Returns complete motor information including config file path
- `getConfigFilePath()` (line 64): Returns appropriate configuration file for motor type
- `registerMotorType()` (line 85): Registers new motor types with product IDs and config files

**Purpose**: Enables automatic motor detection and configuration file selection based on detected motor hardware, supporting seamless multi-motor setups with different motor types.

### 5. motor_sdo_manager.cpp
Handles EtherCAT SDO (Service Data Object) communication for uploading configuration parameters.

**Core SDO Operations:**
- `upload32()` (line 28): Uploads 32-bit parameters with validation and verification
- `uploadFloat()` (line 76): Uploads floating-point parameters with motor format conversion
- `download32()` (line 124): Downloads 32-bit parameters for verification
- `verify32()` (line 144): Verifies uploaded parameters by reading them back
- `verifyFloat()` (line 163): Verifies uploaded floating-point parameters

**High-Level Configuration Upload:**
- `uploadControlGains()` (line 184): Uploads all PID control gains (position, velocity, current)
- `uploadMotorSpecs()` (line 256): Uploads motor specifications (encoder resolution, max speed)
- `uploadSafetyLimits()` (line 289): Uploads position and torque safety limits
- `uploadComplete()` (line 313): Uploads complete configuration in priority order
- `uploadCriticalParameters()` (line 348): Uploads only critical parameters for faster startup

**Internal Helpers:**
- `performSDOWrite()` (line 443): Low-level SOEM SDO write operation
- `performSDORead()` (line 462): Low-level SOEM SDO read operation
- `validateParameter()` (line 489): Validates parameter values for safety
- `floatToMotorFormat()` (line 521): Converts floating-point to motor's internal format

**Purpose**: Provides the communication layer between the configuration system and the motor hardware, ensuring parameters are safely uploaded and verified.

### 6. network_controller.cpp
Implements the EtherCAT network management layer.

**Network Lifecycle:**
- `NetworkController()` (line 19): Initializes EtherCAT master with interface and timing parameters
- `initialize()` (line 46): Initializes EtherCAT master on specified network interface
- `scanNetwork()` (line 76): Scans for EtherCAT slaves on the network
- `configureSlaves()` (line 100): Maps process data and configures distributed clocks
- `startOperation()` (line 149): Transitions all slaves to operational state
- `stopOperation()` (line 232): Safely stops network operation

**Real-Time Communication:**
- `performCommunicationCycle()` (line 287): Executes one EtherCAT communication cycle (send/receive PDO data)
- `flushControlWordUpdate()` (line 556): Forces immediate PDO update for control word changes

**Data Access:**
- `getOutputPDO()` (line 318): Returns output PDO pointer for specific slave
- `getInputPDO()` (line 327): Returns input PDO pointer for specific slave
- `getNetworkStatus()` (line 336): Returns comprehensive network status
- `getSlaveInfo()` (line 363): Returns detailed information about specific slave

**Internal Management:**
- `initializeSlaveInfo()` (line 430): Collects slave information after network scan
- `setupPDOPointers()` (line 459): Maps PDO structures to slave memory regions
- `checkSlaveStates()` (line 372): Verifies all slaves remain operational

**Motor Detection Integration:**
- `getDetectedMotorInfo()` (line 500): Returns motor type information for specific slave
- `getAllDetectedMotors()` (line 531): Returns motor information for all detected slaves

**Purpose**: Manages the complete EtherCAT network infrastructure, providing real-time communication and slave management services to motor controllers.

### 7. robot_controller.cpp
Provides multi-network coordination for complex robotic systems.

**Network Management:**
- `RobotController()` (line 19): Initializes robot controller with global timing parameters
- `addNetwork()` (line 42): Adds new EtherCAT network with configuration validation
- `removeNetwork()` (line 86): Safely removes network from robot system
- `getNetwork()` (line 107): Returns network controller by name

**Robot-Level Operations:**
- `initializeAllNetworks()` (line 134): Initializes all networks in parallel
- `startAllNetworks()` (line 192): Starts operation on all networks with coordinated timing
- `updateAllNetworks()` (line 278): Performs parallel communication cycles across all networks
- `emergencyStopAll()` (line 330): Emergency stop across entire robot system
- `shutdownAll()` (line 358): Graceful shutdown of all networks

**Status and Diagnostics:**
- `allNetworksOperational()` (line 416): Checks if all networks are operational
- `getRobotStatus()` (line 432): Returns comprehensive robot system status
- `getAllNetworkStatus()` (line 458): Returns status of all individual networks

**Internal Coordination:**
- `updateNetworkParallel()` (line 524): Updates individual network in parallel thread
- `updateTimingStatistics()` (line 548): Tracks timing performance across networks
- `validateNetworkConfig()` (line 490): Validates network configuration parameters

**Purpose**: Orchestrates multiple EtherCAT networks with synchronized timing, enabling complex robotic systems with distributed motor control across multiple network segments.

## Control Logic Flow

### Data Flow Architecture

**Configuration Data Flow:**
1. CSV configuration files → MotorConfigParser → MotorController
2. MotorController → SDOManager → EtherCAT SDO → Motor hardware
3. Motor-specific parameters enable adaptive control for different motor types

**Real-Time Control Data Flow:**
1. User commands → MotorController (position/velocity/torque setpoints)
2. MotorController → NetworkController (OutputPDO structures)
3. NetworkController → EtherCAT network → Motor drives
4. Motor drives → EtherCAT network → NetworkController (InputPDO structures)
5. NetworkController → MotorController → User applications (feedback data)

**Safety Data Flow:**
1. Fault detection in MotorController (from InputPDO status)
2. Automatic fault recovery (torque ramp to zero)
3. Emergency stop propagation from RobotController to all networks
4. Parameter validation at SDO upload and real-time command levels

## Initialization Sequence and Program Flow

### System Startup Sequence

**Phase 1: Robot Controller Initialization**
1. `RobotController()` constructor (line 19)
   - Sets global timing parameters (frequency, cycle time)
   - Initializes synchronization primitives and error handling
   - Prepares for multi-network coordination

**Phase 2: Network Configuration**
1. `addNetwork()` (line 42)
   - Validates network configuration (interface name, frequency, etc.)
   - Creates NetworkController instances for each EtherCAT segment
   - Establishes network-specific timing parameters

**Phase 3: Network Infrastructure Setup**
1. `initializeAllNetworks()` (line 134)
   - Parallel initialization of all EtherCAT masters
   - `NetworkController::initialize()` (line 46)
     - Initializes SOEM EtherCAT master on specified interface
     - Allocates IOmap buffer for process data
     - Sets up network communication infrastructure

**Phase 4: EtherCAT Network Discovery**
1. `startAllNetworks()` (line 192)
   - Parallel network scanning and configuration
   - `NetworkController::scanNetwork()` (line 76)
     - Scans for EtherCAT slaves using ec_config_init()
     - Detects all connected motor drives
   - `NetworkController::configureSlaves()` (line 100)
     - Maps process data with ec_config_map()
     - Configures distributed clocks for synchronization
     - Sets up PDO pointers for real-time communication

**Phase 5: Motor Detection and Configuration**
1. `initializeSlaveInfo()` (line 430)
   - Extracts slave information (product ID, vendor ID, PDO sizes)
   - Motor Manager detects motor types from product IDs
   - Maps motor types to appropriate configuration files

2. Motor Controller Configuration:
   - `MotorController::loadConfig()` (line 603)
   - `MotorConfigParser::parseCSV()` (line 88)
     - Parses motor-specific parameters from CSV files
     - Validates critical parameters (encoder resolution, safety limits)
   - `MotorController::uploadConfig()` (line 642)
     - Uploads parameters to motor hardware via SDO

**Phase 6: EtherCAT Operational State**
1. `NetworkController::startOperation()` (line 149)
   - Initializes output PDOs with safe values (zero torque, shutdown state)
   - Transitions all slaves to operational state using CIA402 state machine
   - Verifies operational state and calculates expected working counter
   - Performs initial communication cycle

### Real-Time Operation Loop

**Main Control Cycle (typically 250Hz)**
1. `RobotController::updateAllNetworks()` (line 278)
   - Parallel execution across all networks
   - Maintains global timing synchronization

**Per-Network Communication Cycle**
1. `NetworkController::performCommunicationCycle()` (line 287)
   - ec_send_processdata(): Sends output PDOs to all slaves
   - ec_receive_processdata(): Receives input PDOs from all slaves
   - Updates working counter and error detection

**Per-Motor Control Update**
1. `MotorController::update()` (line 50)
   - Executes active control mode (position, velocity, or torque)
   - Performs safety checks and fault detection
   - Updates control commands based on current mode

**Control Mode Execution:**
- **Torque Mode**: `ramp_torque_command()` (line 501)
  - Implements safe torque ramping to prevent mechanical shock
  - Updates OutputPDO target_torque field
- **Position Mode**: `update_position_command()` (line 534)
  - Limits position steps to prevent excessive motion
  - Handles gear ratio compensation (user commands output shaft, PDO controls motor shaft)
- **Velocity Mode**: `update_velocity_command()` (line 558)
  - Applies gear ratio scaling for proper motor shaft velocities
  - Updates OutputPDO target_velocity field

### Motor Enable Sequence

**CIA402 State Machine Implementation:**
1. `MotorController::enable_motor()` (line 68)
   - Executes state machine transition over multiple cycles
   - `enable_motor_sequence()` (line 412) implements:
     - Step 0: Fault reset if needed (controlword = 0x0080)
     - Step 1: Shutdown (controlword = 0x0006) → Ready to Switch On
     - Step 2: Switch On (controlword = 0x0007) → Switched On
     - Step 3: Enable Operation (controlword = 0x000F) → Operation Enabled

### Safety and Fault Handling

**Fault Detection:**
- Continuous monitoring of InputPDO statusword for fault bits
- `MotorController::has_fault()` (line 344)
- Automatic fault recovery through torque ramp-down

**Emergency Stop Sequence:**
1. `RobotController::emergencyStopAll()` (line 330)
   - Immediate stop across all networks
   - Parallel emergency stop execution
   - Sets emergency stop flag preventing further operation

**Graceful Shutdown:**
1. `RobotController::shutdownAll()` (line 358)
   - Transitions slaves to safe operational state
   - Closes EtherCAT master connections
   - Releases all resources

### Configuration Parameter Flow

**Parameter Upload Sequence:**
1. Motor-specific CSV files define tuning parameters
2. `MotorConfigParser` validates and organizes parameters
3. `MotorSDOManager` uploads in priority order:
   - Priority 1: Safety limits (critical for protection)
   - Priority 2: Control gains (important for performance)
   - Priority 3: Motor specifications (optimization parameters)

**Real-Time Parameter Validation:**
- All control commands undergo safety validation
- Torque commands clamped to motor-specific limits
- Velocity commands checked against maximum safe speeds
- Position commands respect gear ratio calculations

## Key Design Features

### 1. Hierarchical Architecture
- **Multi-layered design** enabling scalability from single motor to complex robot systems
- **Clear separation of concerns** between network management, motor control, and system coordination
- **Modular components** that can be independently tested and maintained

### 2. Real-Time Performance
- **Deterministic timing** with configurable cycle frequencies (typically 250Hz)
- **Parallel network operation** for multi-network systems
- **Optimized PDO communication** with minimal latency
- **Working counter monitoring** for communication integrity

### 3. Safety Systems
- **Multi-level fault detection** from hardware faults to communication errors
- **Automatic fault recovery** with safe torque ramp-down
- **Parameter validation** at configuration upload and runtime
- **Emergency stop propagation** across entire system
- **CIA402 compliant state machines** for safe motor operation

### 4. Configuration Management
- **Motor-agnostic design** supporting multiple motor types (JD8, JD10, JD12)
- **CSV-based configuration** for easy parameter tuning
- **Automatic motor detection** based on EtherCAT product IDs
- **Configuration validation** with fallback defaults
- **Priority-based parameter upload** for critical vs. optimization parameters

### 5. Advanced Control Features
- **Multiple control modes**: position, velocity, and torque control
- **Gear ratio compensation** for output shaft vs. motor shaft coordination
- **Configurable safety limits** for position, velocity, and torque
- **Smooth transitions** between control modes
- **Real-time feedback** with comprehensive status monitoring

## Conclusion

The Draco4 Multi-Motor EtherCAT Control Framework represents a comprehensive solution for high-performance robotic control applications. Its hierarchical architecture, robust safety systems, and configuration-driven approach make it suitable for complex multi-motor systems requiring precise, synchronized control with real-time performance guarantees.

The framework's design emphasizes:
- **Safety first**: Multiple layers of protection and fault handling
- **Deterministic operation**: Real-time guarantees for robotic applications
- **Scalability**: From single motor to multi-network robot systems
- **Maintainability**: Clear architecture with well-defined interfaces
- **Flexibility**: Configuration-driven operation supporting multiple motor types

This makes it an excellent foundation for advanced robotic systems requiring reliable, high-performance motor control.