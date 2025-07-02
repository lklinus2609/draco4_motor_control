# Test Programs

The project includes comprehensive test programs for validation and demonstration:

## Available Test Programs

### Single Motor Tests
- `test_jd8_position` - Position control with gear ratio scaling and 90-degree moves
- `test_jd8_velocity` - Velocity control with precision timing analysis  
- `test_jd8_torque` - Torque control with safety limits and ramping

### Multi-Motor Tests
- `test_multi_motor_simulation` - Multi-motor simulation without hardware
- `test_multi_motor_hardware` - Multi-motor hardware validation

### Multi-Network Tests
- `test_multi_network_simulation` - Multi-network simulation testing
- `test_multi_network_hardware` - Multi-network hardware validation

### Network Tests
- `test_network_controller` - Basic network functionality testing

## Running Tests

### Basic Usage
```bash
sudo ./test_jd8_velocity eth0  # Replace with your interface
sudo ./test_multi_motor_hardware eth0
sudo ./test_multi_network_hardware
```

### Test Descriptions

#### Position Control Test
Tests precise position control with gear ratio calculations:
- 90-degree incremental moves
- Position feedback monitoring
- Gear ratio scaling validation

#### Velocity Control Test
Validates velocity control accuracy:
- RPM command tracking
- Timing precision analysis
- Velocity feedback validation

#### Torque Control Test
Demonstrates safe torque control:
- Torque ramping implementation
- Safety limit enforcement
- Emergency stop functionality

#### Multi-Motor Tests
Validates multiple motor coordination:
- Simultaneous motor control
- Synchronized updates
- Motor detection and configuration

#### Multi-Network Tests
Tests complex robotic systems:
- Multiple EtherCAT networks
- Coordinated network updates
- Network fault handling

## Test Requirements

- Root privileges for EtherCAT access
- Valid network interfaces connected to motors
- SOEM library installation
- Proper motor configuration files