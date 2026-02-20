package frc.robot.subsystems.intake;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {

    // ===== Physics Simulations =====
    private final FlywheelSim m_rollerSim;
    private final ElevatorSim m_slideSim;

    // ===== Commanded Setpoints =====
    private double rollerVoltageCommand = 0.0;
    private double slidePositionCommand = 0.0;

    // ===== Simulated Sensor State =====
    private double simulatedIntakeDistance = Double.POSITIVE_INFINITY; // mm
    private double simulatedIndexerDistance = Double.POSITIVE_INFINITY; // mm

// ===== Simulation Constants =====
private static final double ROTATOR_MOI = 0.001; // kg*m^2 - moment of inertia
private static final double ROTATOR_GEARING = 1.5; // gear ratio
private static final double[] ROTATOR_MEASUREMENT_STD_DEVS = {0.01}; // Standard deviation in rad/s

private static final double SLIDE_CARRIAGE_MASS = 2.0; // kg
private static final double SLIDE_DRUM_RADIUS = 0.02; // meters
private static final double SLIDE_GEARING = 10.0;
private static final double SLIDE_MIN_HEIGHT = 0.0; // meters
private static final double SLIDE_MAX_HEIGHT = 0.3; // meters
private static final double[] SLIDE_MEASUREMENT_STD_DEVS = {0.001}; // Standard deviation in meters
    
private static final double LOOP_PERIOD = 0.02; // 20ms

/**
 * Creates a new IntakeIOSim instance.
 * Initializes physics simulations for roller and slide.
 */
public IntakeIOSim() {
    // Create flywheel sim for roller (spinning intake wheels)
    LinearSystem<N1, N1, N1> m_rollerPlant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), ROTATOR_MOI, ROTATOR_GEARING);
    m_rollerSim = new FlywheelSim(m_rollerPlant , DCMotor.getKrakenX44(1), ROTATOR_MEASUREMENT_STD_DEVS);
  
    // Create elevator sim for slide (linear extension mechanism)
    m_slideSim = new ElevatorSim(
        DCMotor.getKrakenX44(1),      // Motor gearbox: 1x Falcon 500
        SLIDE_GEARING,                // Gear ratio
        SLIDE_CARRIAGE_MASS,          // Mass of carriage
        SLIDE_DRUM_RADIUS,            // Drum/pulley radius
        SLIDE_MIN_HEIGHT,             // Minimum height
        SLIDE_MAX_HEIGHT,             // Maximum height
        true,                         // Simulate gravity
        0.0,                          // Starting position (retracted)
        SLIDE_MEASUREMENT_STD_DEVS    // Measurement noise
    );

    // Initialize with a game piece nearby for testing
    simulatedIntakeDistance = 500.0; // 500mm away
}

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // ===== Update Physics Simulations =====
        
        // Update roller with commanded voltage
        m_rollerSim.setInputVoltage(rollerVoltageCommand);
        m_rollerSim.update(LOOP_PERIOD);

        // Update slide with position control (simplified - real would use PID)
        double slideVoltage = calculateSlideVoltageForPosition(slidePositionCommand);
        m_slideSim.setInputVoltage(slideVoltage);
        m_slideSim.update(LOOP_PERIOD);

        // ===== Simulate Game Piece Collection =====
        simulateGamePieceMovement();

        // ===== Populate Roller Inputs =====
        inputs.rollerVelocityRPS = m_rollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerAppliedVolts = rollerVoltageCommand;
        inputs.rollerCurrentAmps = m_rollerSim.getCurrentDrawAmps();
        inputs.rollerTempCelsius = 25.0; // Simulated temp (could model heating)

        // ===== Populate Slide Inputs =====
        // Convert meters to rotations for consistency with hardware
        double slidePositionMeters = m_slideSim.getPositionMeters();
        inputs.slidePositionRotations = slidePositionMeters / (SLIDE_DRUM_RADIUS * 2 * Math.PI);
        inputs.slideVelocityRPS = m_slideSim.getVelocityMetersPerSecond() / (SLIDE_DRUM_RADIUS * 2 * Math.PI);
        inputs.slideAppliedVolts = slideVoltage;
        inputs.slideCurrentAmps = m_slideSim.getCurrentDrawAmps();
        inputs.slideTempCelsius = 25.0;

        // ===== Populate Sensor Inputs =====
        // inputs.intakeDistance = simulatedIntakeDistance;
        // inputs.intakeTarget = simulatedIntakeDistance <= IntakeSubsystem.INTAKE_THRESHOLD;
    }

    /**
     * Simple proportional control to move slide to desired position.
     * Real hardware uses MotionMagic, but this is good enough for sim.
     */
    private double calculateSlideVoltageForPosition(double targetRotations) {
        double targetMeters = targetRotations * (SLIDE_DRUM_RADIUS * 2 * Math.PI);
        double currentMeters = m_slideSim.getPositionMeters();
        double error = targetMeters - currentMeters;
        
        // Simple P controller (kP = 50 works okay for elevator sim)
        double voltage = error * 50.0;
        
        // Clamp to ±12V
        return Math.max(-12.0, Math.min(12.0, voltage));
    }

    /**
     * Simulates game piece movement through the intake system.
     * When roller spins and piece is close, "intake" it into indexer.
     */
    private void simulateGamePieceMovement() {
        boolean rollerSpinning = Math.abs(m_rollerSim.getAngularVelocityRPM()) > 10.0;
        boolean intakeExtended = m_slideSim.getPositionMeters() > 0.1; // >10cm extended
        // boolean pieceAtIntake = simulatedIntakeDistance <= IntakeSubsystem.INTAKE_THRESHOLD;
        
        // If conditions met, move piece from intake → indexer
        // if (rollerSpinning && intakeExtended && pieceAtIntake) {
        //     simulatedIndexerDistance = 50.0; // 50mm - piece now in indexer
        //     simulatedIntakeDistance = Double.POSITIVE_INFINITY; // no longer at intake
        // }
    }

    // ========== Command Methods ==========
    // These just store commands, actual physics happens in updateInputs()

    @Override
    public void setRollerSpeed(double speed) {
        // Convert percent output (-1 to 1) to voltage
        rollerVoltageCommand = speed * 12.0;
    }

    @Override
    public double getRollerVolts() {
        return rollerVoltageCommand;
    }

    @Override
    public void stopRoller() {
        rollerVoltageCommand = 0.0;
    }

    @Override
    public void setSlidePosition(double position) {
        slidePositionCommand = position;
    }

    @Override
    public double getSlidePosition() {
        // Return current position in rotations
        double positionMeters = m_slideSim.getPositionMeters();
        return positionMeters / (SLIDE_DRUM_RADIUS * 2 * Math.PI);
    }

    // @Override
    // public double getIntakeDistance() {
    //     return simulatedIntakeDistance;
    // }

    // @Override
    // public boolean intakeTargetClose() {
    //     return simulatedIntakeDistance <= IntakeSubsystem.INTAKE_THRESHOLD;
    // }

}