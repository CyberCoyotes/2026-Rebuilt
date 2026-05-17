## Code Commented Out

Scratchpad of commented out code. Guess I'm afraid to let it go!


## RobotContainer.java
        // // POV cycles through LED animations (for testing / manual override)
        // operator.povUp().onTrue(ledSub.cycleNext());
        // operator.povDown().onTrue(ledSub.cyclePrev());


// =====================================================================
// NOTE: The following are potential additional commands and triggers that can be implemented as needed.
// These would go before the Constructor
// =====================================================================

    // Defensive braking if needed
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); 
    
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // LED State Triggers
        // Shooting, intake, and default-idle patterns live here when re-enabled.

    // Shooting — any shoot preset (driver RT, driver POV left, operator A/B/X/Y)
    // Trigger anyShootHeld = driver.rightTrigger(0.5)
    //     .or(driver.povLeft())
    //     .or(operator.a())
    //     .or(operator.b())
    //     .or(operator.x())
    //     .or(operator.y());
        // anyShootHeld
        //     .onTrue(ledSub.showShooting())
        //     .and(RobotModeTriggers.teleop()).onFalse(ledSub.showIdle());


    // // Intaking — driver or operator left trigger
        // Trigger anyIntakeHeld = driver.leftTrigger(0.5)
        //     .or(operator.leftTrigger(0.5));
        // anyIntakeHeld
        //     .onTrue(ledSub.showIntaking())
        //     .and(RobotModeTriggers.teleop()).onFalse(ledSub.showIdle());

    // // Default to idle when enabled and nothing else is active
        // RobotModeTriggers.teleop()
        //     .onTrue(ledSub.showIdle());
        // =====================================================================
        // LED GAME TELEMETRY TRIGGERS (commented out — enable when needed)
        // Requires: gameDataTelemetry accessible here, DriverStation import
        // =====================================================================

        // -- Robot alliance color on enable --
        // new Trigger(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red)
        //     .onTrue(ledSub.showDefault()); // swap showDefault() for a showAllianceRed() if you add one

        // -- Active hub: which alliance is currently in scoring mode --
        // new Trigger(gameDataTelemetry::isRedHubActive)
        //     .onTrue(/* ledSub.showRedHub() */null)
        //     .onFalse(/* ledSub.showDefault() */null);

        // new Trigger(gameDataTelemetry::isBlueHubActive)
        //     .onTrue(/* ledSub.showBlueHub() */null)
        //     .onFalse(/* ledSub.showDefault() */null);

        // -- FMS data received (lights up once auto-scoring data arrives ~3s after auto) --
        // new Trigger(gameDataTelemetry::isDataReceived)
        //     .onTrue(/* ledSub.showAllianceColor() */null);

        // Operator holds a face button to override with a named preset.

        // Auto-reverse: intake running + shooter idle + ball detected in chute = premature ball → reverse indexer
        // Sensor in new place so this probably NOT valid anymore
        // new Trigger(() ->
        //     intake.isRollerRunning() &&
        //     shooter.getState() == ShooterSubsystem.ShooterState.IDLE &&
        //     indexer.isFuelDetected()
        // ).whileTrue(indexer.reverse());