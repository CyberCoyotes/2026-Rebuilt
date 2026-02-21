package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    // ===== IO Layer =====
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    // ===== Intake Constants =====
    // Mechanical limits for the slide
    final static double SLIDE_MIN_POSITION = 0;
    final static double SLIDE_MAX_POSITION = 44.455;
    static final double SLIDE_RETRACTED_POSITION = 0.0;
    
    // See the config for software limit
    static final double SLIDE_EXTENDED_POSITION  = 44.44; // TODO Verify slide position

    final static int INTAKE_THRESHOLD = 1000; // mm, around four inches
    final static double JAM_CURRENT_THRESHOLD = 80.0;

 


    final static double ROLLER_VOLTS = 6.0; // Voltage for intaking fuel

    // Slide voltage constants — used while MotionMagic is disabled
    // Positive extends, negative retracts. Adjust if slide moves too fast/slow.
    final static double SLIDE_EXTEND_VOLTS = 10.0;   // TODO: Tune if needed
    final static double SLIDE_RETRACT_VOLTS = -3.0; // TODO: Tune if needed

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Intake", inputs); // FIXME: enable when AdvantageKit wired up
    }

     /**
     * Sets the current state for dashboard display.
     */
    // public void setState(String state) {
    //     this.currentState = state;
    // }

    // =========================================================================
    // LOW LEVEL METHODS
    // =========================================================================

    public void runRoller() {
        io.setRollerVoltage(ROLLER_VOLTS);
    }

    public void reverseRoller() {
        io.setRollerVoltage(-ROLLER_VOLTS);
    }

    /**
     * Stops the roller motor.
     */
    public void stopRoller() {
        io.setRollerVoltage(0);
    }

    /**
     * Position control via MotionMagic — currently disabled pending tuning.
     * Use extendSlides() / retractSlides() for voltage-based control instead.
     */
    public void setSlidePosition(double position) {
        // io.setSlidePosition(position); // MotionMagic — re-enable once tuned
    }

    /**
     * Extends the intake slides using voltage control.
     * Motor runs at SLIDE_EXTEND_VOLTS until stopSlide() is called.
     * The slide encoder tracks position so MotionMagic can be re-enabled later.
     */
    public void extendSlides() {
        io.setSlideVoltage(SLIDE_EXTEND_VOLTS);
    }

    /**
     * Retracts the intake slides using voltage control.
     * Motor runs at SLIDE_RETRACT_VOLTS until stopSlide() is called.
     * The slide encoder tracks position so MotionMagic can be re-enabled later.
     */
    public void retractSlides() {
        io.setSlideVoltage(SLIDE_RETRACT_VOLTS);
    }

    /**
     * Stops the slide motor.
     * Always call this after extending or retracting to prevent continuous motor run.
     */
    public void stopSlide() {
        io.setSlideVoltage(0);
    }

    // =========================================================================
    // GETTERS
    // =========================================================================

    public double getRollerVoltage() {
        return io.getRollerVoltage();
    }

    // ── Slide State Queries ────────────────────────────────────────────────────
    public double getSlidePosition() {
        return inputs.slidePositionRotations;
    }

    public boolean isSlideExtended() {
        return Math.abs(inputs.slidePositionRotations - SLIDE_EXTENDED_POSITION) < 0.05;
    // =========================================================================
    // COMMANDS
    // =========================================================================

    public Command runRollerCommand() {
        return Commands.startEnd(
                this::runRoller,
                this::stopRoller, this);
    }

    public boolean isSlideRetracted() {
        return Math.abs(inputs.slidePositionRotations - SLIDE_RETRACTED_POSITION) < 0.05;
    }

    /**
     * Main intake command — extends slides and runs roller while held.
     * On release: stops roller. Slides remain extended intentionally.
     * Bind to Left Trigger with whileTrue().
     */
    public Command intakeFuel() {
        return Commands.startEnd(
                () -> {
                    extendSlides();
                    runRoller();
                },
                this::stopRoller,
                this);
    }

    /**
     * Retracts the intake slides using voltage control.
     * Stops the slide motor once retracted position is reached.
     * Bind to Left Bumper with whileTrue() — hold until slides are fully retracted.
     *
     * TODO: Once MotionMagic is tuned, replace with position-based retract
     * so the driver doesn't need to hold the button.
     */
    public Command retractSlidesCommand() {
        return Commands.startEnd(
                this::retractSlides,
                this::stopSlide,
                this);
    }

    /**
     * Ejects fuel by reversing the roller while held.
     * Bind with whileTrue() — roller stops on release.
     *
     * NOTE: Currently commented out — uncomment in RobotContainer when needed.
     */
    // public Command ejectFuel() {
    //     return Commands.startEnd(
    //             this::reverseRoller,  // Fixed: was incorrectly calling this::ejectFuel (recursive)
    //             this::stopRoller,
    //             this);
    // }

    public Command stopFuel() {
        return Commands.runOnce(
                () -> {
                    this.stopRoller();
                    this.retractSlides();
                },
                this
        );
    }

} // end of class
