package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  // public static final CANBus RIO_CANBUS = new CANBus("rio");
  public static final CANBus RIO_CANBUS = CANBus.roboRIO("rio"); // native rio bus

  /*
   * Comment convention for this file:
   * ToDo = value is possibly workable for now, but still needs tuning or
   * verification.
   * FixMe = no safe assumptions or something is legit broken
   */

  // =====================================================================
  // CAN ID Quick Reference
  // =====================================================================

  // candrive (CANivore) bus
  /*
   * Check TunerConstants.java to confirm
   * 
   * 1 FL Drive Motor Kraken X60 (kFrontLeftDriveMotorId)
   * 2 FR Steer Motor Kraken X60 (kFrontRightSteerMotorId)
   * 3 FL CANcoder (kFrontLeftEncoderId)
   * 4 FR Drive Motor Kraken X60 (kFrontRightDriveMotorId)
   * 5 FL Steer Motor Kraken X60 (kFrontLeftSteerMotorId)
   * 6 FR CANcoder (kFrontRightEncoderId)
   * 7 BL Drive Motor Kraken X60 (kBackLeftDriveMotorId)
   * 8 BL Steer Motor Kraken X60 (kBackLeftSteerMotorId)
   * 9 BL CANcoder (kBackLeftEncoderId)
   * 10 BR Drive Motor Kraken X60 (kBackRightDriveMotorId)
   * 11 BR Steer Motor Kraken X60 (kBackRightSteerMotorId)
   * 12 BR CANcoder (kBackRightEncoderId)
   * 14 Pigeon 2 IMU (kPigeonId)
   */

  // Rio bus
  /*
   * 15 CANdle LEDs (Led.CANDLE_ID)
   * 20 Intake Roller Left Kraken X44 (Intake.ROLLER_LEFT_MOTOR_ID)
   * 21 Intake Roller Right Kraken X44 (Intake.ROLLER_RIGHT_MOTOR_ID)
   * 22 Intake Slide Kraken X44 (Intake.SLIDE_MOTOR_ID)
   * 23 Kicker Left Kraken X60 (Indexer.KICKER_LEFT_MOTOR_ID)
   * 24 Kicker Right Kraken X60 (Indexer.KICKER_RIGHT_MOTOR_ID)
   * 25 Flywheel Left Kraken X60 (Shooter.FLYWHEEL_LEFT_MOTOR_ID)
   * 26 Flywheel Right Kraken X60 (Shooter.FLYWHEEL_RIGHT_MOTOR_ID)
   * 27 Conveyor Kraken X44 (Indexer.CONVEYOR_MOTOR_ID)
   * 28 Hood Minion/FXIS (Shooter.HOOD_MOTOR_ID)
   * 42 Chute ToF CANrange (Indexer.CHUTE_TOF_ID)
   */

  public static final double DRIVE_CLAMP = 0.75;

  public static final double HUB_TO_CENTER = Units.inchesToMeters(23.5);

  public static final double ROBOT_WIDTH = Units.inchesToMeters(27);

  public static final double BUMPER_THICKNESS = Units.inchesToMeters(3.75);

  public static final double LL_TO_FRONT = -Vision.LL_FORWARD + (ROBOT_WIDTH / 2);


  // =====================================================================
  // Intake
  // =====================================================================
  public static final class Intake {
    private Intake() {
    }

    /* Kraken X44 with TalonFX controller (x3) */
    public static final int ROLLER_LEFT_MOTOR_ID = 20;
    public static final int ROLLER_RIGHT_MOTOR_ID = 21;
    public static final int SLIDE_MOTOR_ID = 22;

    // Slide setpoints and tuning values
    public static final double SLIDE_RETRACTED_POS = 0.0;
    public static final double SLIDE_HOME_POS = 19.18;
    public static final double SLIDE_EXTENDED_POS = 60.00; //
    public static final double SLIDE_MAX_POS = 65.75; // Confirmed 4-6-26
    public static final double SLIDE_ROLLER_SAFE_MARGIN = 1.5;
    public static final double SLIDE_ROLLER_SAFE_POS = SLIDE_HOME_POS + SLIDE_ROLLER_SAFE_MARGIN;
    public static final double SLIDE_TOLERANCE = 0.05;
    public static final double SLIDE_INCREMENTAL_RETRACT_ROTATIONS = 15.0;
    public static final double SLIDE_MANUAL_STEP_ROTATIONS = 5.0;
    public static final double SLIDE_MANUAL_REPEAT_SECONDS = 0.15;
    public static final double SLIDE_PUMP_OUT_POS = 60.0;
    public static final double SLIDE_PUMP_IN_POS = 40.0;
    public static final double SLIDE_FUEL_COMPRESSION_WAIT_SECONDS = 1; // Was 1.5
    // 0.5 too quick
    // 1.0 still seems quick

    public static final double SLIDE_FUEL_COMPRESSION_DURATION_SECONDS = 6.0;
    public static final double SLIDE_FUEL_PUMP_WAIT_SECONDS = 3.0;
    public static final double SLIDE_FUEL_PUMP_OUT_SECONDS = 0.5;
    public static final double SLIDE_FUEL_PUMP_IN_SECONDS = 0.5;
    public static final double SLIDE_FUEL_PUMP_SENSOR_TIMEOUT_SECONDS = 5.0;

    /*
     * Adjust these motion magic values for normal slide modes.
     * Normal Motion Magic values for moving the slide quickly to position.
     * Add an end-of-line "Tuned" note when confirmed.
     */
    public static final double SLIDE_MM_CRUISE_VELOCITY = 80;
    public static final double SLIDE_MM_ACCELERATION = 80;
    public static final double SLIDE_MM_JERK = 0.0;

    /*
     * Adjust these motion magic values for slow slide modes.
     * Slow Motion Magic values for moving the slide slowly to position.
     * Add an end-of-line "Tuned" note when confirmed.
     */
    public static final double SLIDE_SLOW_MM_CRUISE_VELOCITY = 8; // 20
    public static final double SLIDE_SLOW_MM_ACCELERATION = 16; // 40
    public static final double SLIDE_SLOW_MM_JERK = 0.0;

    public static final double ROLLER_FORWARD_VOLTS = 8;
    public static final double ROLLER_REVERSE_VOLTS = -11; 

    public static final class RollerLeaderConfig {
      private RollerLeaderConfig() {
      }

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

      /* Intake roller limits */
      public static final double SUPPLY_CURRENT_LIMIT = 30.0;
      public static final double STATOR_CURRENT_LIMIT = 40.0;
    }

    public static final class RollerFollowerConfig {
      private RollerFollowerConfig() {
      }

      // While in follower mode, direction is governed by the Follower request — no
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final double SUPPLY_CURRENT_LIMIT = 30.0;
      public static final double STATOR_CURRENT_LIMIT = 40.0;
      public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    }

    public static final class SlideConfig {
      private SlideConfig() {
      }

      /*
       * Changed from Brake to Coast. Idea that slide will push in some vs than
       * breaking trying to hold position.
       */
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

      /* Intake slide Limits */
      public static final double SUPPLY_CURRENT_LIMIT = 30.0;
      // public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
      // public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0;
      public static final double STATOR_CURRENT_LIMIT = 60.0;
      public static final double REVERSE_SOFT_LIMIT = SLIDE_RETRACTED_POS;
      public static final double FORWARD_SOFT_LIMIT = SLIDE_MAX_POS;
      public static final double PEAK_FORWARD_VOLTAGE = 12.0;
      public static final double PEAK_REVERSE_VOLTAGE = -12.0;

      /*
       * Initial PID values for slide position control.
       * Tune to minimize overshoot and oscillation while being snappy
       * Add an end-of-line "Tuned" note when confirmed.
       */
      public static final double KS = 0.0;
      public static final double KV = 0.50; // Tuned 4-8-2026
      public static final double KP = 8.0; // Tuned 4-9-2026
      public static final double KD = 0.0;
      public static final double KA = 0.0;
      public static final double KI = 0.0;

    }

  }

  // =====================================================================
  // Indexer
  // =====================================================================
  public static final class Indexer {

    private Indexer() {
    }

    // Kraken X60 with TalonFX controller (x2); feeds game pieces to shooter
    public static final int KICKER_RIGHT_MOTOR_ID = 23;
    public static final int KICKER_LEFT_MOTOR_ID = 24;

    // Kraken X44 with TalonFX controller; conveyor motor moves pieces along hopper
    public static final int CONVEYOR_MOTOR_ID = 27;

    // CANrange Time of Flight sensor; detects presence of fuel at indexer-kicker
    public static final int CHUTE_TOF_ID = 42;

    /*
     * Test with empty hopper, light hopper load,
     * and full hopper load.
     * Visually assess performance for now.
     * Add a "Tuned" note when each value is confirmed.
     */
    // Conveyor voltage setpoints for feeding fuel to the shooter.
    public static final double CONVEYOR_FORWARD_VOLTAGE = 5; // Was 4 before Q4 
    // Was 2
    // at 6.5 was struggling, increased to 8 to try to help with feeding the
    // shooter, but may need to be tuned down || Tuned 4-8-2026
    public static final double CONVEYOR_REVERSE_VOLTAGE = -6;

    // Probably don't need anymore
    public static final double CONVEYOR_POPPER_VOLTAGE = 1.0;

    /*
     * Test with empty hopper, solo games, light hopper load,
     * and full hopper load.
     * Visually assess performance for now.
     * Add a "Tuned" note when each value is confirmed.
     */

    // Kicker-indexer voltage setpoints for feeding fuel to the shooter.
    public static final double KICKER_FORWARD_VOLTAGE = 5.0; // was 2.5 on Saturday
    public static final double KICKER_REVERSE_VOLTAGE = -4.0;
    public static final double KICKER_POPPER_VOLTAGE = 3.0;

    /*
     * Add a "Tuned" note when each value is confirmed.
     * 
     * Used as the CANrange ProximityThreshold so the hardware "detected" signal
     * matches the same boundary.
     */
    public static final double FUEL_SIZE = 0.1524; // Fuel-Ball is ~6 inches
    public static final double CHUTE_MAX_DISTANCE = 0.6096; // 24" = 0.6096 m.

    // 60% of fuel size to account for sensor variance and ensure reliable detection
    public static final double FUEL_DETECTION_THRESHOLD = FUEL_SIZE * 0.60;
    public static final double FUEL_DETECTION_DISTANCE = CHUTE_MAX_DISTANCE - FUEL_DETECTION_THRESHOLD; // ~10 inches

    /*
     * Verify fuel clear time based on how long it takes for fuel to move through
     * the chute after detection.
     * NOT a Friday priority testing, but should be verified before the next
     * competition.
     */

    // Time to wait after detecting fuel at the chute before considering it
    // "cleared".
    public static final double FUEL_CLEAR_TIME = 2.0; // seconds

    public static final class ConveyorConfig {
      private ConveyorConfig() {
      }

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

      /* Conveyor Limits */
      public static final double SUPPLY_CURRENT_LIMIT = 35.0;
      public static final double STATOR_CURRENT_LIMIT = 40.0;
      public static final double PEAK_FORWARD_VOLTAGE = 12.0;
      public static final double PEAK_REVERSE_VOLTAGE = -12.0;
    }

    public static final class KickerLeaderConfig {
      private KickerLeaderConfig() {
      }

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

      /* Kicker Limits */
      public static final double SUPPLY_CURRENT_LIMIT = 50.0;
      public static final double STATOR_CURRENT_LIMIT = 90.0;
      public static final double PEAK_FORWARD_VOLTAGE = 12.0;
      public static final double PEAK_REVERSE_VOLTAGE = -12.0;
    }

    public static final class KickerFollowerConfig {
      private KickerFollowerConfig() {
      }

      // While in follower mode, direction is governed by the Follower request — no
      // INVERTED needed
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final double SUPPLY_CURRENT_LIMIT = 50.0;
      public static final double STATOR_CURRENT_LIMIT = 90.0;
      public static final double PEAK_FORWARD_VOLTAGE = 12.0;
      public static final double PEAK_REVERSE_VOLTAGE = -12.0;
      public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    }

    public static final class ChuteSensorConfig {
      private ChuteSensorConfig() {
      }

      /*
       * Verify the CANrange hysteresis and FOV settings against the final chute
       * mounting location. Add an end-of-line "Tuned" note when each value is
       * confirmed.
       */
      public static final double PROXIMITY_HYSTERESIS = 0.025;
      public static final double FOV_RANGE_X = 6.75;
      public static final double FOV_RANGE_Y = 6.75;
    }

  }

  // =====================================================================
  // Flywheel
  // =====================================================================
  public static final class Flywheel {
    private Flywheel() {
    }

    // Kraken X60s with TalonFX controller (leader) and (follower)
    public static final int FLYWHEEL_LEFT_MOTOR_ID = 25;
    public static final int FLYWHEEL_RIGHT_MOTOR_ID = 26;

    // Probably not needed anymore
    public static final double POPPER_RPM = 650;
    public static final double POPPER_HOOD = 4.20;
    
    // Kraken X60 free speed
    public static final double MAX_RPM = 6000.0;

    public static final double IDLE_RPM = 0;

    /* 70% of close shot RPM */ 
    public static final double STANDBY_RPM = Shooter.CLOSE_RPM * 0.70; 

    /*
     * Reverse Flywheel RPM for jam clearing.
     * Only reached through eject(), which gates on EJECT_MAX_ENTRY_RPM to prevent
     * violent reversal at high speeds.
     */
    public static final double EJECT_RPM = -1500;
    public static final double EJECT_MAX_ENTRY_RPM = 500.0;

    public static final double TEST_INCREMENT_RPM = 100.0;

    /*
     * These values should be tuned without a game piece
     * These should be done in the prescribed order
     * Use Pheonix Tuner X to set values and graph results
     * Give Chat bot written feedback on the results or use a screenshot
     * Adjust as needed until the flywheel is performing well at the target RPMs
     * with minimal overshoot and minimal oscillation
     * See the /docs/tuning-guide_flywheel.md document for the full tuning process
     */

    // Flywheel PID and feedforward gains.
    public static final double KV = 0.132; // Tuned 4-13-2026 0.132
    public static final double KP = 0.055; // Tuned 4-13-2026  0.055
    public static final double KD = 0.000; // Tuned 4-4-2026
    public static final double KA = 0.000; // Tuned 4-4-2026

    public static final double TOLERANCE_PERCENT = 0.05; //

    /* Flywheel limits */
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final double STATOR_CURRENT_LIMIT = 90;

    /*
     * Tune acceleration to shape spin-up, then refine Slot0 gains.
     * Probably ok to wait until Saturday
     */
    public static final double MM_ACCELERATION_RPS_PER_SEC = 200.0; // Tuned 4-4-2026
    public static final double MM_JERK_RPS_PER_SEC_CUBED = 0.0; // Tuned 4-4-2026
    public static final double MM_EXPO_KV = 0.12; // Tuned 4-4-2026
    public static final double MM_EXPO_KA = 0.06; // Tuned 4-4-2026
    public static final double FAR_HOOD = 0;
    public static final double PASS_RPM = 0;

    public static final class LeaderConfig {
      private LeaderConfig() {
      }

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
    }

    public static final class FollowerConfig {
      private FollowerConfig() {
      }

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    }

  }

  // =====================================================================
  // Hood
  // =====================================================================

  public static final class Hood {
    private Hood() {
    }

    // IDs
    // Hood motor - Minion with TalonFXS controller, adjusts shot angle.
    public static final int HOOD_MOTOR_ID = 28;

    // Mechanism setpoints and tuning
    public static final double MIN_POSE = 0.00; // Mechanical limit, also use to set in Configs
    public static final double MAX_POSE = 7.25; // Mechanical limit; also used to set in Configs

    public static final double TOLERANCE_POSE = 0.1;

    // Manual tuning increments used for bring-up and testing.
    public static final double TEST_INCREMENT = 0.2;
    public static final double ACCELERATION = 20;
    public static final double CRUISE_VELOCITY = 40;

    /*
     * Tune these PID hood values if using motion magic
     * Currently not being used
     * NOT a Friday testing priority!
     */
    public static final double KP = 1.2;
    public static final double KI = 0;
    public static final double KD = 0;

    // Hood limits
    public static final double SUPPLY_CURRENT_LIMIT = 30;
    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final double PEAK_FORWARD_VOLTAGE = 8.0;
    public static final double PEAK_REVERSE_VOLTAGE = -8.0;
    public static final double ENCODER_ZERO_POSITION = 0.0;

    public static final class HoodConfig {
      private HoodConfig() {
      }

      public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
      public static final double REVERSE_SOFT_LIMIT = MIN_POSE;
      public static final double FORWARD_SOFT_LIMIT = MAX_POSE;

    }

    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double MANUAL_STEP = 0.2;

    public static final double POPPER_HOOD = 1.0;

    public static final double PASS_HOOD = 0;

  }

  // =====================================================================
  // Shooter Presets - These are combos of flywheel, hood values, and distance
  // =====================================================================
  public static final class Shooter {

    // Bumpers against the hub if possible
    public static final double CLOSE_DISTANCE = Units.inchesToMeters(18)+ HUB_TO_CENTER + LL_TO_FRONT;
    public static final double CLOSE_RPM = 2350;
    public static final double CLOSE_HOOD = 2.25;

    // Side Bumpers against the tower
    public static final double TOWER_DISTANCE = Units.inchesToMeters(107) + HUB_TO_CENTER + LL_TO_FRONT;
    public static final double TOWER_RPM = 3050;
    public static final double TOWER_HOOD = 4.50;

    // In the trench, mostly against the wall, but turned slightly towards the hub
    public static final double TRENCH_DISTANCE = Units.inchesToMeters(110) + HUB_TO_CENTER + LL_TO_FRONT; // FIXME Trench distance
    public static final double TRENCH_RPM = 3050; // FIXME Trench RPM
    public static final double TRENCH_HOOD = 5.0; // FIXME Trench hood

    // In a corner by human player station or depot-corner, angled towards the hub
    public static final double FAR_DISTANCE = Units.inchesToMeters(176) + HUB_TO_CENTER + LL_TO_FRONT; // FIXME Far distance
    public static final double FAR_RPM = 3450; // FIXME Far RPM
    public static final double FAR_HOOD = 8.0; // FIXME Far hood
 
    // // Back Bumpers approximately against the driver station wall, angled towards the hub
    // public static final double DRIVER_STATION_DISTANCE = Units.inchesToMeters(182.11) - ROBOT_WIDTH - BUMPER_THICKNESS; // FIXME Driver station distance
    // public static final double DRIVER_STATION_RPM = 3700; // FIXME Driver station RPM
    // public static final double DRIVER_STATION_HOOD = 4.5; // FIXME Driver station hood

    // For passing passing from midfield
    public static final double PASS_RPM = 3000; // FIXME Pass RPM
    public static final double PASS_HOOD = 4.5; // FIXME Pass hood

  }

  // =====================================================================
  // Vision / Limelight
  // =====================================================================
  public static final class Vision {
    private Vision() {
    }

    // Camera configuration
    public static final String LIMELIGHT3_NAME = "limelight-three";
    public static final String LIMELIGHT4_NAME = "limelight-four";

    // Pipeline indices
    public static final int APRILTAG_PIPELINE = 0;
    // public static final int GAME_PIECE_PIPELINE = 1; // Optional: for note/game
    // piece detection

    // =================================================================
    // Camera Mounting
    // =================================================================

    
    // 9.75 inches = 0.24765 meters; negative because camera is behind the reference point
    public static final double LL_FORWARD = -0.24765; 
    
    public static final double LL_RIGHT= 0;

    // 20.75 inches = 0.52705 meters; but camera is mounted low, so using 0 for now until we can verify with measurements
    public static final double LL_UP = 0.52705; 
 
    public static final double LL_ROLL = 0;    
    public static final double LL_PITCH = 15.4;
    public static final double LL_YAW = 0;     
   
    // TODO Consider loosening the alignment tolerance if the robot is having trouble reaching the aligned state
    /** Tolerance for horizontal alignment in degrees used in FuelCommands.java */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 1.00; // TODO 

    /** Minimum target area to consider target valid (prevents false positives) */
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;

    /** Maximum distance to trust vision measurement in meters */
    public static final double MAX_DISTANCE_METERS = 6.0;

    // State tracking
    /** Time in seconds before considering target "lost" after losing sight */
    public static final double TARGET_TIMEOUT_SECONDS = 0.5;

    // Vision-driven drivetrain rotation
    /**
     * Proportional gain for rotational alignment: (rad/s output) per (degree of tx
     * error).
     *
     * - Too low → slow to center, may not reach ALIGNED before timeout
     * - Too high → oscillates left/right around the target
     * See `tuning-guide_vision.md` for step-by-step procedure.
     */

    /*
     * Current mechanical layout: shooter and camera are mounted on the back of the
     * robot and shooter faces backwards
     * Chassis front must point 180° away from the hub when aligning to shoot.
     */

    public final static double ALIGNMENT_OFFSET_DEGREES = 0; // Updated to reflect the rebuild 4-12-2026

    // Keep this moderate
    // aggressive values amplify pose-estimator jitter during alignment.
    public static final double ROTATIONAL_KP = 0.12; // Tuned 4-8-2026

    // TODO: KD for vision not currently used
    // Dampens oscillation; increase if sluggish settling, decrease if jittery
    public static final double ROTATIONAL_KD = 0.005;
    /*
     * Maximum rotational rate the vision command will apply to the drivetrain
     * (rad/s).
     * Default: 3.0 rad/s (~172°/s). Reduce if the robot swings too aggressively.
     */
    public static final double MIN_ALIGNMENT_ROTATION_RAD_PER_SEC = 0.15; // tune to just above static friction
    public static final double MAX_ALIGNMENT_ROTATION_RAD_PER_SEC = 4.5; // tune to prevent overshooting and oscillation

    public static final double ALIGNMENT_DRIVETRAIN_CLAMP = 0.40;

    public static final double MIN_DISTANCE_M = 0.25;
    public static final double MAX_DISTANCE_M = 6.0;

    /*
     * Tune up from 0 — 50 degrees of aim offset per m/s of lateral velocity
     * At this point, its not a high priority
     */
    public static final double LEAD_COMPENSATION_DEG_PER_MPS = 0;

    // == Valid tag IDs =========================
    // NOTE: MIN/MAX here are used for general target validation in VisionSubsystem.
    // Hub-specific filtering uses BLUE_HUB_TAG_IDS / RED_HUB_TAG_IDS arrays below.
    public static final int MIN_VALID_TAG_ID = 1;
    public static final int MAX_VALID_TAG_ID = 28; // Fixed: was -1, which rejected all tags

    // Blue hub AprilTag IDs (2026 field layout — all 8 hub faces)
    // Layout: 18/27 (top chute), 19/20 (sides), 26/25 (sides), 21/24 (bottom chute)
    public static final int[] BLUE_HUB_TAG_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };

    // Red hub AprilTag IDs (2026 field layout — all 8 hub faces)
    // Layout: 8/5 (top chute), 9/10 (sides), 4/3 (sides), 11/2 (bottom chute)
    public static final int[] RED_HUB_TAG_IDS = { 2, 3, 4, 5, 8, 9, 10, 11 };

    // Hub center positions in WPILib blue-origin field coordinates (meters).
    // Used by poseAlignAndShoot / autoAlignAndShoot for odometry-based aiming.
    // Red hub is the field-length mirror of blue: x = 17.548 - 4.625 = 12.923 but
    // Choreo shows 11.923, so using that for now until we can verify with
    // measurements.
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.625, 4.025); // FIXME Verify hub
                                                                                           // locations, 4.625 vs 4.025
    public static final Translation2d RED_HUB_LOCATION = new Translation2d(11.923, 4.025); // FIXME Verify hub
                                                                                           // locations, 4.625 vs 4.025

  }

  // =====================================================================
  // LEDs
  // =====================================================================
  public static final class Led {
    private Led() {
    }

    /** CANdle device ID */
    // public static final int CANDLE_ID = 15;

    /** WS2811 logical segments per meter */
    public static final int SEGMENTS_PER_METER = 20;

    /** WS2811 physical LEDs per logical segment */
    public static final int LEDS_PER_SEGMENT = 36;

    /** Total strip length in meters */
    public static final double STRIP_LENGTH_METERS = 0.28575;

    /** Total number of logical addressable segments */
    public static final double LOGICAL_LED_COUNT = SEGMENTS_PER_METER * STRIP_LENGTH_METERS;

  }

  // =====================================================================
  // Auto
  // =====================================================================
  public static final class Auto {
    private Auto() {
    }

    // How long to wait after driving before doing something else
    public static final double DRIVE_WAIT = 1.0;
    public static final double SCORE_WAIT = 1.0;

  }
}
