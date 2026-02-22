package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

@SuppressWarnings("unused") // Suppress warnings for unused right now

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    // ===== State Tracking =====
    private String currentState = "IDLE";

    // ===== Intake Constants =====
    final static int INTAKE_THRESHOLD = 1000; // mm, around four inches
    final static double JAM_CURRENT_THRESHOLD = 80.0;

    // Mechanical limits for the slide
    final static double SLIDE_MIN_POSITION = 0;
    final static double SLIDE_MAX_POSITION = 1.85;
    final static double SLIDE_RETRACTED_POSITION = 0.0;
    final static double SLIDE_EXTENDED_POSITION = 1.91;

    final static double ROLLER_VOLTS = 6.0; // Voltage for intaking fuel

    // Slide voltage constants — used while MotionMagic is disabled
    // Positive extends, negative retracts. Adjust if slide moves too fast/slow.
    final static double SLIDE_EXTEND_VOLTS = 10.0;   // TODO: Tune if needed
    final static double SLIDE_RETRACT_VOLTS = -3.0; // TODO: Tune if needed

    /** Roller voltage used during retract — slow reverse to push pieces inwards */
    final static double RETRACT_REVERSE_ROLLER_VOLTS = 2.0; // TODO: Tune if needed

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SmartDashboard.putNumber("Intake/SlidePosition", io.getSlidePosition());
        SmartDashboard.putBoolean("Intake/RollerRunning", io.getRollerVolts() != 0);
      
        // Logger.processInputs("Intake", inputs); // FIXME
    }

    /**
     * Sets the current state for dashboard display.
     */
    public void setState(String state) {
        this.currentState = state;
    }

    // =========================================================================
    // LOW LEVEL METHODS
    // =========================================================================

    public void setRollerSpeed(double volts) {
        io.setRollerSpeed(volts);
    }

    public void runRoller() {
        io.setRollerSpeed(ROLLER_VOLTS);
    }

    public void stopRoller() {
        io.setRollerSpeed(0);
    }

    public void reverseRoller() {
        io.setRollerSpeed(-ROLLER_VOLTS);
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

    public double getRollerVolts() {
        return io.getRollerVolts();
    }

    public double getSlidePosition() {
        return io.getSlidePosition();
    }

    // =========================================================================
    // COMMANDS
    // =========================================================================

    public Command runRollerCommand() {
        return Commands.startEnd(
                this::runRoller,
                this::stopRoller, this);
    }

    public Command stopRollerCommand() {
        return Commands.run(this::stopRoller, this);
    }

    /**
     * Main intake command — extends slides and runs roller at full ROLLER_VOLTS while held.
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
     * Auto intake command — extends slides and runs roller at a custom voltage.
     * Use this in auto routines where a different roller speed is desired.
     * The regular intakeFuel() is unchanged for teleop use.
     *
     * @param rollerVolts Voltage to run the roller at (e.g. 4.0 for slower auto intake)
     */
    public Command intakeFuel(double rollerVolts) {
        return Commands.startEnd(
                () -> {
                    extendSlides();
                    io.setRollerSpeed(rollerVolts);
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
     * Retracts the intake slides while slowly reversing the roller.
     * Useful for pushing any partially-intaked pieces back out during retract.
     * On release: stops both the slide motor and the roller.
     * Bind to Left Bumper with whileTrue().
     */
    public Command retractAndReverseRollerCommand() {
        return Commands.startEnd(
                () -> {
                    retractSlides();
                    io.setRollerSpeed(RETRACT_REVERSE_ROLLER_VOLTS);
                },
                () -> {
                    stopSlide();
                    stopRoller();
                },
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