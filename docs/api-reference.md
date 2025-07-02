# API Reference

---
[Home](index.md) | [API Reference](api-reference.md) | [Developer Guide](developer-guide.md) | [Usage Examples](usage-examples.md)
---

## NetworkController

### Network Management
```cpp
bool initialize()                    // Initialize EtherCAT master
bool scanNetwork()                   // Scan for slaves
bool configureSlaves()              // Configure discovered slaves  
bool startOperation()               // Start operational mode
void stopOperation()                // Stop operation
void performCommunicationCycle()    // Execute one communication cycle
```

### Network Status
```cpp
bool isOperational()                // Check if network is operational
int getSlaveCount()                 // Get number of slaves
```

## MotorController

### Motor Control
```cpp
bool enable_motor()                 // Enable motor via CIA402 state machine
bool disable_motor()                // Disable motor
void update()                       // Update motor control (call at configured frequency)
```

### Control Commands  
```cpp
bool set_velocity_rpm(int rpm)      // Set velocity command
bool set_position_counts(int32_t)   // Set position command (output shaft)
bool set_torque_millinm(int16_t)    // Set torque command
```

### Feedback
```cpp
double getMotorRPM()                // Get motor shaft velocity
double getOutputRPM()               // Get output shaft velocity
int32_t getPosition()               // Get motor shaft position  
int32_t getOutputPos()              // Get output shaft position
int16_t getTorque()                 // Get actual torque
uint16_t getStatus()                // Get CIA402 status word
```

### Configuration and Timing
```cpp
double getUpdateFrequencyHz()       // Get configured frequency
int getCycleTimeUs()                // Get cycle time in microseconds
```

## MotorManager

### Initialization
```cpp
bool initialize()                   // Initialize and detect motors
bool initializeMotors()             // Initialize detected motors
```

### Motor Access
```cpp
std::vector<std::shared_ptr<MotorController>> getMotors()  // Get all motors
MotorController* getMotor(int index)                       // Get specific motor
```

### Bulk Operations
```cpp
void updateAll()                    // Update all motors
void enableAll()                    // Enable all motors
void disableAll()                   // Disable all motors
```

## RobotController

### Network Management
```cpp
bool addNetwork(const std::string& name, const std::string& interface)
bool initializeAll()               // Initialize all networks
void updateAll()                   // Update all networks synchronously
void stopAll()                     // Stop all networks
```

### Motor Access
```cpp
std::vector<std::shared_ptr<MotorController>> getMotors(const std::string& network)
std::vector<std::shared_ptr<MotorController>> getAllMotors()
```