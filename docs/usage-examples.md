# Usage Examples

---
[Home](index.md) | [API Reference](api-reference.md) | [Developer Guide](developer-guide.md) | [Code Analysis](code-analysis.md) | [Usage Examples](usage-examples.md)
---

## Multi-Motor Control

```cpp
#include "motor_manager.hpp"

// Create motor manager for automatic detection
synapticon_motor::MotorManager manager("eth0", 250.0);

// Initialize and detect motors
if (!manager.initialize()) {
    std::cerr << "Manager initialization failed" << std::endl;
    return 1;
}

// Get detected motors
auto motors = manager.getMotors();
std::cout << "Detected " << motors.size() << " motors" << std::endl;

// Control multiple motors
for (auto& motor : motors) {
    motor->set_velocity_rpm(500);  // Set all to 500 RPM
}

// Real-time control loop
while (running) {
    manager.updateAll();  // Updates all motors automatically
    std::this_thread::sleep_for(std::chrono::microseconds(4000)); // 250Hz
}
```

## Multi-Network Robot Control

```cpp
#include "robot_controller.hpp"

// Create robot controller for complex systems
synapticon_motor::RobotController robot(250.0);

// Add multiple networks
robot.addNetwork("left_arm", "eth0");
robot.addNetwork("right_arm", "eth1");
robot.addNetwork("base", "eth2");

// Initialize all networks
if (!robot.initializeAll()) {
    std::cerr << "Robot initialization failed" << std::endl;
    return 1;
}

// Get motors by network
auto left_motors = robot.getMotors("left_arm");
auto right_motors = robot.getMotors("right_arm");

// Synchronized control
while (running) {
    robot.updateAll();  // Synchronized update of all networks
    std::this_thread::sleep_for(std::chrono::microseconds(4000));
}
```

## Position Control Example

```cpp
// Set position control mode
motor.set_position_counts(90000);  // 90 degrees with gear ratio

// Monitor position feedback
while (!position_reached) {
    motor.update();
    network->performCommunicationCycle();
    
    int32_t current_pos = motor.getOutputPos();
    std::cout << "Position: " << current_pos << " counts" << std::endl;
    
    std::this_thread::sleep_for(std::chrono::microseconds(4000));
}
```

## Torque Control with Safety

```cpp
// Enable torque ramping for safety
motor.enableTorqueRamping(true);

// Set torque limit
motor.setTorqueLimit(5000);  // 5 Nm limit

// Gradual torque increase
for (int torque = 0; torque <= 3000; torque += 100) {
    motor.set_torque_millinm(torque);
    
    // Update and wait
    motor.update();
    network->performCommunicationCycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
```