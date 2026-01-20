package frc.robot.subsystems.intake2;

/**
 * IntakeIOSim - Simulation implementation of IntakeIO for testing without hardware.
 *
 * This class provides fake intake data for testing robot logic without real motors.
 * Useful for:
 * - Testing intake state machine logic
 * - Testing commands that use intake
 * - Simulation mode
 * - Rapid prototyping at home
 *
 * FEATURES:
 * - Simulated rotator velocity
 * - Simulated slide position with realistic movement
 * - Methods to control simulation state for testing
 */
public class IntakeIOSim implements IntakeIO {

    // ===== Simulated State =====
    private double rotatorPercent = 0.0;
    private double currentSlidePosition = 0.0;
    private double targetSlidePosition = 0.0;

    // ===== Simulation Parameters =====
    private static final double SLIDE_SPEED_ROTATIONS_PER_CYCLE = 0.5;  // Rotations per 20ms cycle
    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double ROTATOR_CURRENT_PER_PERCENT = 20.0;  // Amps at 100% output
    private static final double SLIDE_CURRENT = 5.0;  // Amps when moving
    private static final double MOTOR_TEMP = 40.0;  // Celsius

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // Simulate slide movement toward target
        if (currentSlidePosition < targetSlidePosition) {
            currentSlidePosition = Math.min(currentSlidePosition + SLIDE_SPEED_ROTATIONS_PER_CYCLE,
                                            targetSlidePosition);
        } else if (currentSlidePosition > targetSlidePosition) {
            currentSlidePosition = Math.max(currentSlidePosition - SLIDE_SPEED_ROTATIONS_PER_CYCLE,
                                            targetSlidePosition);
        }

        // Rotator data (fake velocity based on percent output)
        inputs.rotatorVelocityRPS = rotatorPercent * 100.0;  // Fake: 100 RPS at full power
        inputs.rotatorAppliedVolts = Math.abs(rotatorPercent) * MOTOR_VOLTAGE;
        inputs.rotatorCurrentAmps = Math.abs(rotatorPercent) * ROTATOR_CURRENT_PER_PERCENT;
        inputs.rotatorTempCelsius = MOTOR_TEMP;

        // Slide data
        inputs.slidePositionRotations = currentSlidePosition;
        inputs.slideVelocityRPS = (currentSlidePosition != targetSlidePosition) ?
                                  SLIDE_SPEED_ROTATIONS_PER_CYCLE / 0.02 : 0.0;  // Convert to RPS
        inputs.slideAppliedVolts = (currentSlidePosition != targetSlidePosition) ? MOTOR_VOLTAGE : 0.0;
        inputs.slideCurrentAmps = (currentSlidePosition != targetSlidePosition) ? SLIDE_CURRENT : 0.0;
        inputs.slideTempCelsius = MOTOR_TEMP;
    }

    @Override
    public void setRotator(double percent) {
        this.rotatorPercent = percent;
    }

    @Override
    public void setSlidePosition(double rotations) {
        this.targetSlidePosition = Math.max(0, rotations);  // Clamp to non-negative
    }

    @Override
    public void stop() {
        this.rotatorPercent = 0.0;
        // Slide stays at current position when stopped
    }

    @Override
    public void zeroSlide() {
        this.currentSlidePosition = 0.0;
        this.targetSlidePosition = 0.0;
    }

    // ===== Simulation Control Methods =====

    /**
     * Instantly sets the slide to target position (for testing).
     * Bypasses realistic movement simulation.
     */
    public void setSlideInstant(double rotations) {
        this.currentSlidePosition = rotations;
        this.targetSlidePosition = rotations;
    }

    /**
     * Gets the current simulated slide position.
     */
    public double getCurrentSlidePosition() {
        return currentSlidePosition;
    }

    /**
     * Gets the current simulated rotator output.
     */
    public double getRotatorPercent() {
        return rotatorPercent;
    }
}
