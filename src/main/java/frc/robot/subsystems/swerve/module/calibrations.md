Modules:
-----------------------
- [x] Enter modules in the order specified in ModuleUtils.ModulePosition

ModuleConstants: (real and simulation)
-----------------------
- [ ] Wheel Diameter
- [ ] Coupling Ratio
- [ ] Velocity At 12 Volts
- [x] Modules locations (in meters)

Encoder:
----------------------
- [x] Encoder ID
- [x] Sensor Range (should be PlusMinusHalf)
- [x] Sensor Direction (should be CounterClockwise in default sds module)

Steer:
-----------------------
- [x] Motor ID
- [ ] Inverted
- [x] Neutral Mode
- [x] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [x] Encoder Usage and ID (should use fuse)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [x] Use ContinuousWrap
- [x] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [x] Moment of inertia
- [x] DCMotor

Drive:
-----------------------
- [x] Motor ID
- [ ] Inverted
- [x] Neutral Mode
- [ ] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [x] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [x] Moment of inertia
 - [x] DCMotor