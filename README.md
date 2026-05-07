# Final Year Project - Jumping Wheel-legged Robot

## Hardware (Mechanical)
- Leg Structure: five-bar linkage
  - Link 1 (Ground): 72 mm
  - Link 2 & 3 (Thigh): 100 mm
  - Link 4 & 5 (Shank): 150 mm
- Body Weight (Without Legs): 2.1 kg
- Motors 1 & 2 are the joints' control
  - installed back-to-back on the same side
- Motors 3 & 4 are the wheel
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
2. Turning on the robot will auto-reset the current position to zero readings.
  - Assume turning on the robot at 95 mm height from the base to the floor
  - Zero position's angle is 10.93032737696 degrees

## Joint Position Model
- Zero pose: motor driver reading near 0 turns.
- Default pose is a common angle offset from the zero pose:
  - kDefaultFromZeroDeg
- The controller uses zero pose as the logical joint-angle reference.
- Changing the default pose does not require changing the other joint parameters.
- Motor 2 mirrors motor 1 in joint-angle space (opposite sign).

## Safety Angle Limits
- Limits are defined relative to the zero pose:
  - kMinJointAngleDeg
  - kMaxJointAngleDeg
- These limits stay fixed even if the default pose changes.

## Controller Behavior
- Menu button: enable drive
- View button: disable drive (wheels idle, legs stay in pose)
- A button: jump sequence
  - Hold A: crouch (sneak target)
  - Release A: launch (jump target), then recover to default pose
  - Short cooldown prevents accidental retrigger
- The sneak/jump targets are stored relative to the zero pose and stay independent of the default pose.
- Left joystick: control robot motion
  - X-axis: turn left/right
  - Y-axis: move forward/backward (speed target)
- RB: turbo scale for faster motion

## Control (PID + Mixing)
- Balance target is a fixed pitch setpoint (kPitchSetpointDeg).
- Speed loop: left-stick Y -> speed target -> speed PID -> desired pitch offset.
- Balance loop: pitch + pitch-rate damping -> wheel velocity command.
- Turn loop: left-stick X -> turn target -> turn PID -> differential wheel velocity.
- Forward authority is reduced when tilt error grows to prevent runaway.

## Hardware Checking
1. Controller Checking
  - Loop the checking function until a signal is received
2. IMU Checking
  - Firmware now fails fast if MPU6050 init fails.
  - Boot log should show either:
    - IMU initialization OK.
    - IMU initialization failed (check wiring/power/SDA/SCL).
3. Motors Checking
  - Check the CAN bus signal from motor 1 - 4 by sequence
    - Poll encoder estimates at a limited rate to avoid bus flooding

## Starting Setup
Assume starting on a flat floor/surface
1. Hardware Checking (see "Hardware Checking")
2. Check IMU coordinates
3. Start the PID system and set the motors 1 & 2 to the default position
4. Stay balanced and wait for the Xbox controller's signal (see "Controller Behavior")

## Remark
This is the code for the Arduino UNO R4 Minima.
The code for ESP32: https://github.com/SuperInoch/FYP_JumpingWheelLeggedRobot_Controller
