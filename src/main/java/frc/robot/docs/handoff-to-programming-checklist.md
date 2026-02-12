## Robot First Power-On ToDo (Programming)

- [ ] Confirm the latest code is deployed to the roboRIO and the Driver Station laptop is on the correct team network.
- [ ] Verify robot firmware and vendor library versions match the project (WPILib + Phoenix + other vendordeps).
- [ ] Set up and verify CAN IDs in Phoenix Tuner X for all CTRE devices (example: TalonFX, CANcoder, Pigeon).
- [ ] Label each CAN device in Phoenix Tuner so names match subsystem conventions used in code.
- [ ] Check CAN bus health (utilization, bus errors, missing devices) before enabling mechanisms.
- [ ] Verify motor controller neutral modes (Brake/Coast) are correct for each subsystem.
- [ ] Confirm all motor inversions in hardware match expected software behavior.
- [ ] Validate sensor directions and zero points (encoders, absolute encoders, gyro yaw).
- [ ] Run a safe "bump test" for each motor one-at-a-time (low output, wheels off ground where possible).
- [ ] Verify swerve module/drive orientation and steering angle offsets before any path following.
- [ ] Test Driver Station controller mappings and confirm no stale bindings or deadband issues.
- [ ] Verify all limit switches, beam breaks, and safety interlocks report correctly in telemetry.
- [ ] Bring up logging (AdvantageKit/NT) and ensure key signals are visible during test mode.
- [ ] Tune current limits, ramp rates, and PID gains starting with conservative values.
- [ ] Validate brownout behavior and monitor battery sag under step-load tests.
- [ ] Confirm robot disable behavior is safe (motors stop, pneumatics hold/release as intended).
- [ ] Test autonomous init and a short, low-risk auto routine with clear e-stop access.
- [ ] Record findings/issues in a startup checklist doc and open follow-up tasks for anything abnormal.