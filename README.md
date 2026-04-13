# Final Year Project - Jumping Wheel-legged Robot

## Hardware (Mechanical)
- Leg Structure: five-bar linkage
  - Link 1 (Ground): 72 mm
  - Link 2 & 3 (Thigh): 100 mm
  - Link 4 & 5 (Shank): 150 mm
- Body Weight (Without Legs): 2.1 kg
- Motor 1 & 2 are the joints control
  - installed back-to-back on the same side
- Motor 3 & 4 are the wheel
  - installed back-to-back

## Hardware (Electrical)
- Battery: 24V 12A LiPo
- Board: Arduino Uno R4 Minima (Main); ESP32 (For Bluetooth Connection)
- IMU: MPU6050 (I2C)
- Motor: GIM6010 with GDS68 motor driver
- CAN secondary bitrate: 250000 bps
- Joint motors:
  - Motor 1
    - node ID: 0x01
    - control mode: position
    - left side of the robot
  - Motor 2
    - node ID: 0x02
    - control mode: position
    - right side of the robot
  - Motor 3
    - node ID: 0x03
    - control mode: speed
    - left side of the robot
  - Motor 4
    - node ID: 0x04
    - control mode: speed
    - right side of the robot
- Controller: Xbox Controller

## Motor Calibration Workflow (for motor 1 & 2)
1. Set the robot for the ideal zero pose.
2. Turn on the motor will auto reset the current position to zero reading.
  - Assume turning on the robot at 95 mm height from the base to floor
  - Zero position's angle is 10.93032737696 degrees

## Joint Position Model
- Zero pose: motor driver reading near 0 turns.
- Defualt pose is a common angle offset from zero pose:
  - kDefaultFromZeroDeg
- Motor 2 mirrors motor 1 in joint-angle space (opposite sign).

## Safety Angle Limits
- Limits are defined relative to zero pose:
  - kMinJointAngleDeg
  - kMaxJointAngleDeg

## Controller Behavior
- Press "A": the robot sneak
  - motor 1: rotate anti-clockwise to the sneak position
  - motor 2: rotate clockwise to the sneak position
- Release "A": the robot jump
  - motor 1: rotate clockwise to the jump position
  - motor 2: rotate anti-clockwise to the jump position
- Left joystick: control the robot's moving direction
  - x-axis: turn left / right
  - y-axis: move forward / backward

## Hardware Checking
1. Controller Checking
  - Loop the checking function until signal received
2. IMU Checking
  - Firmware now fails fast if MPU6050 init fails.
  - Boot log should show either:
    - IMU initialization OK.
    - IMU initialization failed (check wiring/power/SDA/SCL).
  - Runtime telemetry includes imu:ok or imu:fail.
3. Motors Checking
  - Check the CAN bus signal from motor 1 - 4 by sequence
    - Poll encoder estimates at a limited rate to avoid bus flooding

## Starting Setup
Assume starting on a falt floor/surface
1. Hardware Checking (see "Hardware Checking")
2. Check IMU coordinates
3. Start PID system and set the motor 1 & 2 to the standard position
4. Stay balance and wait for Xbox controller's singal (see "Controller Behavior")