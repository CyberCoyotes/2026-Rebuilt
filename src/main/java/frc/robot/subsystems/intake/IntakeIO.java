package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Status;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/**
 * IntakeIO - Hardware abstraction interface for the intake subsystem.
 *
 * The intake collects game pieces from the ground using:
 * - 1x Rotator motor: Spins intake wheels to pull in game pieces
 * - 1x Slide motor: Extends/retracts intake mechanism
 *
 * HARDWARE:
 * - 1x TalonFX (rotator) - percent output control
 * - 1x TalonFX (slide) - position control
 *
 * PATTERN: IO Interface
 * - IntakeIO: Interface defining what intake hardware can do
 * - IntakeIOTalonFX: Real hardware implementation with CTRE motors
 * - IntakeIOSim: Simulation for testing without hardware
 *
 * @author @Isaak3
 */
public interface IntakeIO {

    /**
     * IntakeIOInputs - Container for all intake sensor data.
     *
     * This class holds all data we read from the intake hardware each cycle.
     * Implements LoggableInputs for automatic AdvantageKit logging and replay.
     */
    class IntakeIOInputs implements LoggableInputs {
        // ===== Rotator Motor Data =====
        /** Rotator motor velocity in rotations per second */
        public double rotatorVelocityRPS = 0.0;

        /** Rotator motor applied voltage */
        public double rotatorAppliedVolts = 0.0;

        /** Rotator motor supply current in amps */
        public double rotatorCurrentAmps = 0.0;

        /** Rotator motor temperature in Celsius */
        public double rotatorTempCelsius = 0.0;

        // ===== Slide Motor Data =====
        /** Slide position in rotations (0 = retracted) */
        public double slidePositionRotations = 0.0;

        /** Slide motor velocity in rotations per second */
        public double slideVelocityRPS = 0.0;

        /** Slide motor applied voltage */
        public double slideAppliedVolts = 0.0;

        /** Slide motor supply current in amps */
        public double slideCurrentAmps = 0.0;

        /** Slide motor temperature in Celsius */
        public double slideTempCelsius = 0.0;

        // Sensor Data
        // distance from nearest thing to intake sensor in mm
        public double intakeDistance = 0.0;

        //is a target detected 
        public boolean intakeTarget = false;

        //distance from nearest thing to indexer in mm
        public double indexerDistance = 0.0;

        public boolean indexerTarget = false;


        @Override
        public void toLog(LogTable table) {
            // Rotator motor
            table.put("RotatorVelocityRPS", rotatorVelocityRPS);
            table.put("RotatorAppliedVolts", rotatorAppliedVolts);
            table.put("RotatorCurrentAmps", rotatorCurrentAmps);
            table.put("RotatorTempCelsius", rotatorTempCelsius);

            // Slide motor
            table.put("SlidePositionRotations", slidePositionRotations);
            table.put("SlideVelocityRPS", slideVelocityRPS);
            table.put("SlideAppliedVolts", slideAppliedVolts);
            table.put("SlideCurrentAmps", slideCurrentAmps);
            table.put("SlideTempCelsius", slideTempCelsius);

            //sensors
            table.put("intakeSensorDistance", intakeDistance);
            table.put("intakeTargetDetected", intakeTarget);
            table.put("indexerSensorDistance", indexerDistance);
            table.put("indexerTargetDetected", indexerTarget);
        }

        @Override
        public void fromLog(LogTable table) {
            // Rotator motor
            rotatorVelocityRPS = table.get("RotatorVelocityRPS", rotatorVelocityRPS);
            rotatorAppliedVolts = table.get("RotatorAppliedVolts", rotatorAppliedVolts);
            rotatorCurrentAmps = table.get("RotatorCurrentAmps", rotatorCurrentAmps);
            rotatorTempCelsius = table.get("RotatorTempCelsius", rotatorTempCelsius);

            // Slide motor
            slidePositionRotations = table.get("SlidePositionRotations", slidePositionRotations);
            slideVelocityRPS = table.get("SlideVelocityRPS", slideVelocityRPS);
            slideAppliedVolts = table.get("SlideAppliedVolts", slideAppliedVolts);
            slideCurrentAmps = table.get("SlideCurrentAmps", slideCurrentAmps);
            slideTempCelsius = table.get("SlideTempCelsius", slideTempCelsius);

            //sensors
            intakeDistance = table.get("intakeSensorDistance", intakeDistance);
            intakeTarget = table.get("intakeTargetDetected", intakeTarget);
            indexerDistance = table.get("indexerSensorDistance", indexerDistance);
            indexerTarget = table.get("indexerTargetDetected", indexerTarget);
        }
    }

    /**
     * Updates the inputs object with current hardware readings.
     * Called every robot loop (20ms) by the subsystem.
     *
     * @param inputs Object to populate with current sensor values
     */
    void updateInputs(IntakeIOInputs inputs); //took out default because these methods are fine abstract

    //rotator methods
    void setRotatorSpeed(double velocity);
    StatusSignal<Voltage> getRotatorVolts();

    //slide methods
    void setSlidePosition(double position);
    StatusSignal<Angle> getSlidePosition();

    //intake sensor methods
    double getIntakeDistance();
    boolean intakeTargetClose();

    //indexer sensor methods
    double getIndexerDistance();
    boolean indexerTargetClose();

    //multi-hardware methods
    void toRestingState();
    boolean isJammed();

}
