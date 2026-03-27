package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  private Constants() {
  }

  /** CTRE CAN bus name (empty string means "rio") */
  // public static final CANBus RIO_CANBUS = new CANBus("rio");
  public static final CANBus RIO_CANBUS = CANBus.roboRIO("rio"); // native rio bus

  /*
   * Comment convention for this file:
   * ToDo = value is intentional for now, but still needs on-robot tuning or verification.
   * FixMe = metadata, naming, or assumptions are currently known to be wrong or incomplete.
   */

  // =========================================================
  // CAN ID Quick Reference — keep in sync with source constants ##
  // =========================================================
  /*
  === candrive bus (CANivore) ===
    1 FL Drive Motor  Kraken X60 (TunerConstants.kFrontLeftDriveMotorId)
    2 FR Steer Motor  Kraken X60 (TunerConstants.kFrontRightSteerMotorId)
    3 FL CANcoder     (TunerConstants.kFrontLeftEncoderId)
    4 FR Drive Motor  Kraken X60 (TunerConstants.kFrontRightDriveMotorId)
    5 FL Steer Motor  Kraken X60 (TunerConstants.kFrontLeftSteerMotorId)
    6 FR CANcoder     (TunerConstants.kFrontRightEncoderId)
    7 BL Drive Motor  Kraken X60 (TunerConstants.kBackLeftDriveMotorId)
    8 BL Steer Motor  Kraken X60 (TunerConstants.kBackLeftSteerMotorId)
    9 BL CANcoder     (TunerConstants.kBackLeftEncoderId)
    10 BR Drive Motor Kraken X60 (TunerConstants.kBackRightDriveMotorId)
    11 BR Steer Motor Kraken X60 (TunerConstants.kBackRightSteerMotorId)
    12 BR CANcoder    (TunerConstants.kBackRightEncoderId)
    14 Pigeon 2 IMU   (TunerConstants.kPigeonId)
    15 CANdle LEDs    (Led.CANDLE_ID)
  
  === rio bus ===
    20 Intake Roller Left   Kraken X44 (Intake.ROLLER_LEFT_MOTOR_ID)
    21 Intake Roller Right  Kraken X44 (Intake.ROLLER_RIGHT_MOTOR_ID)
    22 Intake Slide         Kraken X44 (Intake.SLIDE_MOTOR_ID)
    23 Kicker Left          Kraken X60 (Indexer.KICKER_LEFT_MOTOR_ID)
    24 Kicker Right         Kraken X60 (Indexer.KICKER_RIGHT_MOTOR_ID)
    25 Flywheel Left        Kraken X60 (Shooter.FLYWHEEL_LEFT_MOTOR_ID)
    26 Flywheel Right       Kraken X60 (Shooter.FLYWHEEL_RIGHT_MOTOR_ID)
    27 Conveyor             Kraken X44 (Indexer.CONVEYOR_MOTOR_ID)
    28 Hood                 Minion/FXIS (Shooter.HOOD_MOTOR_ID)
    42 Chute ToF CANrange   (Indexer.CHUTE_TOF_ID)
  */

  // =========================================================
  // Intake
  // =========================================================
  public static final class Intake {
    private Intake() {
    }

    // == Shared / startup assumptions =========================
    public static final double ENCODER_ZERO_POSITION = 0.0;

    // == IDs ===============================================

    /*
     * Kraken X44 with TalonFX controller (x3)
     * FIXME: Update Intake motor names in Phoenix Tuner to match these code-side IDs.
     */
    public static final int ROLLER_LEFT_MOTOR_ID = 20;
    public static final int ROLLER_RIGHT_MOTOR_ID = 21;
    public static final int SLIDE_MOTOR_ID = 22;

    // == Mechanism setpoints =================================
    /*
     * TODO: Verify all slide positions and Motion Magic values after the latest
     * mechanical changes. Add an end-of-line "Tuned" note when each value is confirmed.
     */
    public static final double SLIDE_RETRACTED_POS = 0.0;
    public static final double SLIDE_EXTENDED_POS = 44.40;
    public static final double SLIDE_MAX_POS = 44.454;
    public static final double SLIDE_TOLERANCE = 0.25;
    public static final double SLIDE_INCREMENTAL_RETRACT_ROTATIONS = 15.0;

    public static final double SLIDE_MM_CRUISE_VELOCITY = 32;
    public static final double SLIDE_MM_ACCELERATION = 32;
    public static final double SLIDE_MM_JERK = 0.0;

    public static final double SLIDE_SLOW_MM_CRUISE_VELOCITY = 4.0;
    public static final double SLIDE_SLOW_MM_ACCELERATION = 4.0;

    // Agitation positions used while shooting / pumping fuel.
    public static final double SLIDE_PUMP_OUT_POS = 40.0;
    public static final double SLIDE_PUMP_IN_POS = 30.0;

    /*
     * TODO: Verify roller voltages on the current mechanism. These are working
     * estimates and should be rechecked after roller or intake geometry changes.
     */
    public static final double ROLLER_FORWARD_VOLTS = 11.0;
    public static final double ROLLER_REVERSE_VOLTS = -8.0;

    public static final class RollerConfig {
      private RollerConfig() {
      }

      // == Hardware config ====================================
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake; // TODO: Consider Coast
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
      public static final double SUPPLY_CURRENT_LIMIT = 40.0;
      public static final double STATOR_CURRENT_LIMIT = 40.0;
      public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    }

    public static final class SlideConfig {
      private SlideConfig() {
      }

      // == Hardware config ====================================
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast; // TODO: Changing from Brake to Coast; especially if it gets hit
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

      /*
       * TODO: Verify slide current limits, soft limits, and closed-loop gains on the
       * updated mechanism. Add an end-of-line "Tuned" note when each value is confirmed.
       */
      public static final double SUPPLY_CURRENT_LIMIT = 40.0;
      public static final double STATOR_CURRENT_LIMIT = 40.0;
      public static final double REVERSE_SOFT_LIMIT = SLIDE_RETRACTED_POS;
      public static final double KP = 2.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KS = 0.7;
    }

  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {

    private Indexer() {
    }

    // == IDs ===============================================
    /*
     * Kraken X60 with TalonFX controller (x2); feeds pieces to shooter.
     * FIXME: Update kicker and conveyor device names in Phoenix Tuner to match code.
     */
    public static final int KICKER_LEFT_MOTOR_ID = 23;
    public static final int KICKER_RIGHT_MOTOR_ID = 24;

    // Kraken X44 with TalonFX controller; conveyor motor moves pieces along hopper
    public static final int CONVEYOR_MOTOR_ID = 27;

    // CANrange Time of Flight sensor; detects presence of fuel at indexer-kicker
    public static final int CHUTE_TOF_ID = 42;

    // == Mechanism outputs ==================================
    /* 
     * TODO: Verify conveyor and kicker voltages on the current hopper / shooter
     * geometry. These were carried forward from earlier mechanical revisions.
     */
    public static final double CONVEYOR_FORWARD_VOLTAGE = 6.0;
    public static final double CONVEYOR_REVERSE_VOLTAGE = -4.0;
    public static final double CONVEYOR_POPPER_VOLTAGE = 3.0;

    public static final double KICKER_FORWARD_VOLTAGE = 8.0;
    public static final double KICKER_REVERSE_VOLTAGE = -8.0;
    public static final double KICKER_POPPER_VOLTAGE = 3.0; 

    // == Sensor geometry / thresholds ========================
    /* Used as the CANrange ProximityThreshold so the hardware "detected" signal matches the same boundary.
     * Software threshold for fuel detection.
     * A fuel ball is ~6 in (0.1524 m)
     * Anything below ~10 in (0.25 m) means fuel is present in the chute.
     */
    public static final double FUEL_SIZE = 0.1524; // ~10 inches
    public static final double CHUTE_MAX_DISTANCE = 0.36; // TODO: Re-measure for the current chute / shooter width.
    public static final double FUEL_DETECTION_THRESHOLD = FUEL_SIZE * 0.75; // 75% of fuel size to account for sensor variance and ensure reliable detection
    public static final double FUEL_DETECTION_DISTANCE = CHUTE_MAX_DISTANCE - FUEL_DETECTION_THRESHOLD; // ~10 inches

    // Time to wait after detecting fuel at the chute before considering it "cleared".
    public static final double FUEL_CLEAR_TIME = 2.0; // seconds

    public static final class ConveyorConfig {
      private ConveyorConfig() {
      }

      // == Hardware config ====================================
      /*
       * TODO: Verify conveyor current limits, polarity, and voltage caps against the
       * current hardware. Add an end-of-line "Tuned" note when each value is confirmed.
       */
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
      public static final double SUPPLY_CURRENT_LIMIT = 40.0;
      public static final double STATOR_CURRENT_LIMIT = 40.0;
      public static final double PEAK_FORWARD_VOLTAGE = 12.0;
      public static final double PEAK_REVERSE_VOLTAGE = -12.0;
    }

    public static final class KickerConfig {
      private KickerConfig() {
      }

      // == Hardware config ====================================
      /*
       * TODO: Verify kicker current limits, polarity, and follower alignment on the
       * current roller stackup. Add an end-of-line "Tuned" note when each value is confirmed.
       */
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
      public static final double SUPPLY_CURRENT_LIMIT = 45.0;
      public static final double STATOR_CURRENT_LIMIT = 80.0;
      public static final double PEAK_FORWARD_VOLTAGE = 12.0;
      public static final double PEAK_REVERSE_VOLTAGE = -12.0;
      public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    }

    public static final class ChuteSensorConfig {
      private ChuteSensorConfig() {
      }

      // == Hardware config ====================================
      /*
       * TODO: Verify the CANrange hysteresis and FOV settings against the final chute
       * mounting location. Add an end-of-line "Tuned" note when each value is confirmed.
       */
      public static final double PROXIMITY_HYSTERESIS = 0.025;
      public static final double FOV_RANGE_X = 6.75;
      public static final double FOV_RANGE_Y = 6.75;
    }

  }

  // =========================================================
  // Flywheel
  // =========================================================
  public static final class Flywheel {
    private Flywheel() {
    }

    // == IDs ===============================================
    // Kraken X60 with TalonFX controller (leader); Flywheel A motor
    public static final int FLYWHEEL_LEFT_MOTOR_ID = 25;

    // Kraken X60 with TalonFX controller (follower of A); Flywheel B motor
    public static final int FLYWHEEL_RIGHT_MOTOR_ID = 26;
    
    // == Mechanism setpoints / tuning ========================
    // Kraken X60 free speed
    public static final double MAX_RPM = 6000.0; 
    public static final double IDLE_RPM = 0;

    /**
     * TODO: Verify shot RPMs on the current shooter. Mechanical changes may alter
     * the required speed for each distance. Add an end-of-line "Tuned" note when confirmed.
     */
    public static final double POPPER_RPM = 650;
    public static final double STANDBY_RPM = 1000;
    public static final double CLOSE_RPM = 2700;
    public static final double TOWER_RPM = 3200;
    public static final double TRENCH_RPM = 3200;
    public static final double FAR_RPM = 3800;
    public static final double PASS_RPM = 3603;

    /*
     * Reverse Flywheel RPM for jam clearing. 
     * Only reached through eject(), which gates on EJECT_MAX_ENTRY_RPM.
     */
    public static final double EJECT_RPM = -1500;

    /*
     * Maximum forward flywheel RPM at which EJECT is safe to enter. Prevents
     * violent reversal.
     */
    public static final double EJECT_MAX_ENTRY_RPM = 500.0;
    
    public static final double KP = 0.15;
    public static final double KV = 0.119;
    public static final double KD = 0.001;

    /*
     * TODO: Verify flywheel tolerance against current steady-state variation.
     * Previously 0.10 measured steady-state;
     * Variance ±30 RPM at 3300; 3% = ±99 RPM (~3× variance)
     */
    public static final double TOLERANCE_PERCENT = 0.05;

    public static final double SUPPLY_CURRENT_LIMIT = 50;
    public static final double STATOR_CURRENT_LIMIT = 90;
    public static final double TEST_INCREMENT_RPM = 100.0;
    public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.10; // seconds from neutral to full output

    public static final class LeaderConfig {
      private LeaderConfig() {
      }

      // == Hardware config ====================================
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    }

    public static final class FollowerConfig {
      private FollowerConfig() {
      }

      // == Hardware config ====================================
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    }

  }

  // =========================================================
  // Hood
  // =========================================================
 
  public static final class Hood {
    private Hood() {
    }

    // == IDs ===============================================
    // Hood motor - Minion with TalonFXS controller, adjusts shot angle.
    public static final int HOOD_MOTOR_ID = 28;

    // == Mechanism setpoints / tuning ========================
    public static final double MIN_POSE = 0.00; // Mechanical limit, also use to set in Configs
    public static final double MAX_POSE = 4.356934; // Mechanical limit; also used to set in Configs
    
    // TODO: Verify hood tolerance on the current linkage and backlash.
    public static final double POSE_TOLERANCE = 0.05; 

    /*
     * TODO: Verify hood setpoints on the current shooter geometry. Add an end-of-line
     * "Tuned" note when each value is confirmed.
     */
    public static final double CLOSE_HOOD = 0.00; 
    public static final double POPPER_HOOD = 4.30; //
    public static final double TOWER_HOOD = 4.30;
    public static final double TRENCH_HOOD = 4.30;
    public static final double FAR_HOOD = 4.30; // Was 5.50
    public static final double PASS_HOOD = 2.00;

    // Manual tuning increments used for bring-up and testing.
    public static final double TEST_INCREMENT = 0.2;
    public static final double ACCELERATION = 0;
    public static final double CRUISE_VELOCITY = 0;

    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double SUPPLY_CURRENT_LIMIT = 30;
    public static final double PEAK_REVERSE_VOLTAGE = -4.0;
    public static final double PEAK_FORWARD_VOLTAGE = 4.0;
    public static final double ENCODER_ZERO_POSITION = 0.0;

    public static final class Config {
      private Config() {
      }

      // == Hardware config ====================================
      public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    }
  }

  // =========================================================
  // Vision / Limelight
  // =========================================================
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

    // =========================================================
    // Camera mounting - Primarily Documentation Purposes
    // =========================================================

    /* TODO: Verify camera mounting offsets on the current robot. */

    /* Height of Limelight lens from floor in meters is 19.25 inches = 0.489 meters */
    public static final double CAMERA_HEIGHT_METERS = 0.489;

    /* 
    * Camera is on the back of robot from center reference of Pigeon 2
    * The Shooter is on the back of robot from center reference of Pigeon 2 as well
    * -9.5 inches = 0.2413 meters 
    */ 
    public static final double CAMERA_BACK_OFFSET_METERS = 0.2413;

    // Camera is **now** center
    public static final double CAMERA_LEFT_OFFSET_METERS = 0;

    /** Angle of camera from horizontal in degrees (positive = tilted up) */
    // 25 degrees is a common starting point for angled vision setups, but should be
    // measured for accuracy.
    public static final double CAMERA_ANGLE_DEGREES = 15.5;

    // Alignment tolerances
    /** Tolerance for horizontal alignment in degrees used in FuelCommands.java */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 1.0; // appeared to be a duplicate

    /** Minimum target area to consider target valid (prevents false positives) */
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;

    /** Maximum distance to trust vision measurement in meters */
    public static final double MAX_DISTANCE_METERS = 8.0;

    // State tracking
    /** Time in seconds before considering target "lost" after losing sight */
    public static final double TARGET_TIMEOUT_SECONDS = 0.5;

    // == Vision-driven drivetrain rotation ========================
    /**
     * Proportional gain for rotational alignment: (rad/s output) per (degree of tx
     * error).
     *
     * Tuning starting point: 0.06
     * - Too low → slow to center, may not reach ALIGNED before timeout
     * - Too high → oscillates left/right around the target
     * See TUNING.md §5 for step-by-step procedure.
     */

    // started at 40; now 10;
    public static final double ROTATIONAL_KP = 0.10;

    /**
     * Maximum rotational rate the vision command will apply to the drivetrain
     * (rad/s).
     * Prevents violent snap when tx error is large on first acquisition.
     * Default: 3.0 rad/s (~172°/s). Reduce if the robot swings too aggressively.
     */
    public static final double MAX_ALIGNMENT_ROTATION_RAD_PER_SEC = 5.0;
    public static final double MIN_DISTANCE_M = 0.5;
    public static final double MAX_DISTANCE_M = 8.0;
    // public static final double ALIGNMENT_TOLERANCE_DEG = 0.5; // Duplicate of
    // ALIGNMENT_TOLERANCE_DEGREES above, but with a tighter tolerance for "aligned"
    // state if needed?
    // public static final double MAX_ROT_RAD_PER_SEC = 3.0; // Duplicate of
    // MAX_ALIGNMENT_ROTATION_RAD_PER_SEC above, but with a more aggressive cap if
    // needed?

    public static final double LEAD_COMPENSATION_DEG_PER_MPS = 00; // Tune up from 0 — 50 degrees of aim offset per m/s
                                                                   // of lateral velocity

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
    // Red hub is the field-length mirror of blue: x = 17.548 - 4.625 = 12.923 but Choreo shows 11.923, so using that for now until we can verify with measurements.
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.625, 4.025);
    public static final Translation2d RED_HUB_LOCATION = new Translation2d(11.923, 4.025);

  }

  // =========================================================
  // LEDs
  // =========================================================
  public static final class Led {
    private Led() {
    }

    /** CANdle device ID */
    public static final int CANDLE_ID = 15;

    /** WS2811 logical segments per meter */
    public static final int SEGMENTS_PER_METER = 20;

    /** WS2811 physical LEDs per logical segment */
    public static final int LEDS_PER_SEGMENT = 36;

    /** Total strip length in meters */
    public static final double STRIP_LENGTH_METERS = 0.28575;

    /** Total number of logical addressable segments */
    public static final double LOGICAL_LED_COUNT = SEGMENTS_PER_METER * STRIP_LENGTH_METERS;

  }

  public static final class Auto {
    private Auto() {
    }

     // How long to wait after driving before doing something else
    public static final double DRIVE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less
    public static final double SCORE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less

  }
}
