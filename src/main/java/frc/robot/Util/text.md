```
// ShooterIOHardware — replace bare apply() calls:
PhoenixUtil.applyConfig("Flywheel A", 
    () -> flywheelMotorA.getConfigurator().apply(FlywheelConfig.leader()));
PhoenixUtil.applyConfig("Flywheel B", 
    () -> flywheelMotorB.getConfigurator().apply(FlywheelConfig.follower()));
PhoenixUtil.applyConfig("Flywheel C", 
    () -> flywheelMotorC.getConfigurator().apply(FlywheelConfig.follower()));
PhoenixUtil.applyConfig("Hood",       
    () -> hoodMotor.getConfigurator().apply(HoodConfig.hood()));

// IntakeIOHardware — same pattern:
PhoenixUtil.applyConfig("Roller", 
    () -> roller.getConfigurator().apply(RollerConfig.roller()));
PhoenixUtil.applyConfig("Slide",  
    () -> slide.getConfigurator().apply(SlideConfig.slide()));

// IndexerIOHardware — replace existing applyConfig() helper entirely,
// delete the local helper method, use PhoenixUtil instead:
PhoenixUtil.applyConfig("Conveyor",  
    () -> conveyorMotor.getConfigurator().apply(conveyorConfig()));
PhoenixUtil.applyConfig("Indexer",   
    () -> indexerMotor.getConfigurator().apply(indexerConfig()));
PhoenixUtil.applyConfig("Chute ToF", 
    () -> chuteToF.getConfigurator().apply(chuteCANrangeConfig()));

```