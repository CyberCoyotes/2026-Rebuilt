package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

    // ==== IO Layer ====
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    // Tracks roller direction so getIntakeState() doesn't need to poll the IO layer
    private enum RollerState { STOPPED, RUNNING, REVERSED }
    private RollerState rollerState = RollerState.STOPPED;

    // ==== Intake Constants ====
    static final double SLIDE_RETRACTED_POSITION = 0.0;
    static final double SLIDE_EXTENDED_POSITION  = 44.44; // TODO: Verify on robot
    static final double SLIDE_MIN_POSITION        = 0.0;
    static final double SLIDE_MAX_POSITION        = 44.455;

    static final double ROLLER_VOLTS = 6.0;

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // TODO: Enable when AdvantageKit is wired up
        // Logger.processInputs("Intake", inputs);
    }

    // ==== State Queries ====

    /** True when slide is within tolerance of the extended setpoint. */
    public boolean isSlideFullyExtended() {
        return Math.abs(inputs.slidePositionRotations - SLIDE_EXTENDED_POSITION) < 0.05;
    }

    /** True when slide is within tolerance of the retracted setpoint. */
    public boolean isSlideFullyRetracted() {
        return Math.abs(inputs.slidePositionRotations - SLIDE_RETRACTED_POSITION) < 0.05;
    }

    public String getIntakeState() {
        if (isSlideFullyExtended() && rollerState == RollerState.RUNNING)  
            return "Intaking";
        if (isSlideFullyExtended() && rollerState == RollerState.REVERSED) 
            return "Ejecting";
        if (isSlideFullyExtended())
            return "Extended";
        if (isSlideFullyRetracted())
            return "Retracted";
        return "Moving";
    }

    // ==== LOW-LEVEL METHODS — called by command factories ====
    // Rule: command factories call these methods, never other command factories.

    // Roller
    public void runRoller() {
        io.setRollerVoltage(ROLLER_VOLTS);
        rollerState = RollerState.RUNNING;
    }
    
    public void reverseRoller() {
        io.setRollerVoltage(-ROLLER_VOLTS);
        rollerState = RollerState.REVERSED;
    }

    public void stopRoller(){
        io.stopRoller();
        rollerState = RollerState.STOPPED;
    }

    // Slide — MotionMagic position control (motor holds position after command)
    public void extendSlides(){
        io.setSlidePosition(SLIDE_EXTENDED_POSITION);
        // intakeState
    }
    
    public void retractSlides(){
        io.setSlidePosition(SLIDE_RETRACTED_POSITION);
    }

    public void retractSlidesSlow(){
        io.setSlidePositionSlow(SLIDE_RETRACTED_POSITION);
    }

    public void stopSlide(){
        io.stopSlide();
    }

    // ==== COMMAND FACTORIES ====
    // Rule: build commands from the low-level methods above, never from other commands.

    // ---- Roller commands ----

    /** Runs roller while held; stops on release. */
    public Command intakeRoller() {
        return Commands.startEnd(this::runRoller, this::stopRoller, this)
                .withName("IntakeRoller");
    }

    /** Reverses roller while held; stops on release. */
    public Command reverseIntakeRoller() {
        return Commands.startEnd(this::reverseRoller, this::stopRoller, this)
                .withName("ReverseIntakeRoller");
    }

    // Auton convenience wrappers — build on intakeRoller(), not duplicate logic
    public Command intakeRollerTimer(double seconds) {
        return intakeRoller().withTimeout(seconds);
    }

    public Command intakeRollerUntil(BooleanSupplier condition) {
        return intakeRoller().until(condition);
    }

    // ---- Slide commands ----

    /**
     * Extends slides to the MotionMagic setpoint.
     * Runs once — motor holds position automatically.
     */
    public Command extendSlidesCmd() {
        return Commands.runOnce(this::extendSlides, this)
                .withName("ExtendSlides");
    }

    /**
     * Retracts slides to the MotionMagic setpoint.
     * Runs once — motor holds position automatically.
     */
    public Command retractSlidesCmd() {
        return Commands.runOnce(this::retractSlides, this)
                .withName("RetractSlides");
    }

    // ---- Combination commands ----

    /**
     * While button is held:
     *      Extend the slides to the full out position
     *      Run the roller while button is held.
     * When button is released:
     *      Stop the roller.
     *      Slides stay extended (MotionMagic holds them).
     *
     */
    public Command intakeFuel() {
        return Commands.startEnd(
                () -> {
                    extendSlides();
                    runRoller();
                },
                this::stopRoller,
                this)
                .withName("IntakeFuel");
    }

    // Auton convenience wrappers — build on intakeFuel(), not duplicate logic
    public Command intakeFuelTimer(double seconds) {
        return intakeFuel().withTimeout(seconds);
    }

    // Auton convenience wrappers — build on intakeFuel(), not duplicate logic
    public Command intakeFuelUntil(BooleanSupplier condition) {
        return intakeFuel().until(condition);
    }

    /**
     * Stops roller and retracts slides.
     */
    public Command stopFuel() {
        return Commands.runOnce(
                () -> {
                    stopRoller();
                    retractSlides();
                },
                this)
                .withName("StopFuel");
    }

    /**
     * Similar to stopFuel - stops roller and retracts slides.
     * But slides retract with a a slowler profile
     */
    public Command compressFuel() {
        return Commands.runOnce(
                () -> {
                    stopRoller();
                    retractSlidesSlow();
                },
                this)
                .withName("CompressFuel");
    }

} // end of class