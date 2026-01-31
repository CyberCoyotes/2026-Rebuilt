package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {
  private Constants() {}

  /** CTRE CAN bus name (empty string means "rio") */
  public static final CANBus kCANBus = new CANBus("rio");

  // =========================================================
  // Drive / Swerve
  // =========================================================
  // CAN ID Allocation:
  //   1-12  Drivetrain (drive motors, steer motors, CANcoders) — TunerConstants.java
  //   14    Pigeon 2 (IMU) — TunerConstants.java
  //   15    CANivore / CANdle (LED controller)
  //   20-39 Other motors (intake, indexer, shooter, climber)
  //   40+   Sensors (ToF, CANrange, etc.)
  // CAN Bus: canivore (for drivetrain + CANdle), rio (for other subsystems)
  
  // =========================================================
  // Intake
  // =========================================================
  public static final class Intake {
    private Intake() {}

    /** Intake rotator motor - Kraken X44 with TalonFX controller */
    public static final int INTAKE_ROTATOR_MOTOR_ID = 20;

    /** Intake slide motor A - Kraken X44 with TalonFX controller */
    public static final int INTAKE_SLIDE_A_MOTOR_ID = 21;

    /** Intake slide motor B - Kraken X44 with TalonFX controller (paired with A) */
    public static final int INTAKE_SLIDE_B__MOTOR_ID = 22;

    /** Time of Flight sensor - CANrange, confirms fuel presence */
    public static final int INTAKE_SENSOR_ID = 41;
  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {
    private Indexer() {}

    /** Indexer motor - Kraken X44 with TalonFX controller, feeds pieces to shooter */
    public static final int INDEXER_MOTOR_ID = 23;

    /** Conveyor motor - Kraken X44 with TalonFX controller, moves pieces along hopper */
    public static final int CONVEYOR_MOTOR_ID = 24;

    /** Time of Flight sensor - Detects fuel at indexer exit (optional) */
    public static final int INDEXER_TOF_ID = 42;

    /** Time of Flight sensor A - Detects "fullness" of hopper */
    public static final int HOPPER_TOP_A_TOF_ID = 43;

    /** Time of Flight sensor B - Detects "fullness" of hopper */
    public static final int HOPPER_TOP_B_TOF_ID = 44;

    /** Time of Flight sensor C - Detects "fullness" of hopper */
    public static final int HOPPER_TOP_C_TOF_ID = 45;
  }

  // =========================================================
  // Shooter
  // =========================================================
  public static final class Shooter {
    private Shooter() {}

    /** Flywheel A motor - Falcon 500 with TalonFX controller (leader) */
    public static final int FLYWHEEL_A_MOTOR_ID = 25;

    /** Flywheel B motor - Falcon 500 with TalonFX controller (follower of A) */
    public static final int FLYWHEEL_B_MOTOR_ID = 26;

    /** Flywheel C motor - Falcon 500 with TalonFX controller (follower of A) */
    public static final int FLYWHEEL_C_MOTOR_ID = 27;

    /** Hood motor - Minion with TalonFXS controller, adjusts shot angle */
    public static final int HOOD_MOTOR_ID = 28;

    /** Counter wheel motor - Kraken X44 with TalonFX controller */
    public static final int COUNTER_WHEEL_MOTOR_ID = 29;
  }

  // =========================================================
  // Climber
  // =========================================================
  public static final class Climber {
    private Climber() {}

    /** Climber motor A - Kraken X60 with TalonFX controller */
    public static final int CLIMB_A_MOTOR_ID = 30;

    /** Climber motor B - Kraken X60 with TalonFX controller */
    public static final int CLIMB_B_MOTOR_ID = 31;
  }

  // =========================================================
  // Vision / Limelight
  // =========================================================
  public static final class Vision {
    private Vision() {}

    // Camera configuration
    public static final String LIMELIGHT3_NAME = "limelight3";
    public static final String LIMELIGHT4_NAME = "limelight4";


    // Pipeline indices
    public static final int APRILTAG_PIPELINE = 0;
    public static final int GAME_PIECE_PIPELINE = 1; // Optional: for note/game piece detection

    // Camera mounting (ADJUST THESE FOR YOUR ROBOT!)
    /** Height of Limelight lens from floor in meters */
    public static final double CAMERA_HEIGHT_METERS = 0.5;  // TODO: Measure actual height

    /** Angle of camera from horizontal in degrees (positive = tilted up) */
    public static final double CAMERA_ANGLE_DEGREES = 25.0;  // TODO: Measure actual angle

    // Target heights (from 2024 game manual - update for 2025/2026)
    /** Height of AprilTag center from floor in meters */
    public static final double APRILTAG_HEIGHT_METERS = 1.45;  // TODO: Update for 2026 game

    // Alignment tolerances
    /** Tolerance for horizontal alignment in degrees */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;

    /** Minimum target area to consider target valid (prevents false positives) */
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;

    /** Maximum distance to trust vision measurement in meters */
    public static final double MAX_DISTANCE_METERS = 5.0;

    // Valid tag IDs (from 2024 field - update for 2026)
    public static final int MIN_VALID_TAG_ID = 1;
    public static final int MAX_VALID_TAG_ID = -1;  // TODO: Set to correct max tag ID for 2026 field before competition

    // State tracking
    /** Time in seconds before considering target "lost" after losing sight */
    public static final double TARGET_TIMEOUT_SECONDS = 0.5;
  }

  // =========================================================
  // LEDs
  // =========================================================
  public static final class Led {
    private Led() {}

    /** CANdle device ID */
    public static final int CANDLE_ID = 15;

    /** CAN bus name for the CANdle (typically "canivore") */
    public static final String CAN_BUS_NAME = "canivore"; // 'TODO Adjust if needed'

    /** WS2811 logical segments per meter */
    public static final int SEGMENTS_PER_METER = 20;

    /** WS2811 physical LEDs per logical segment */
    public static final int LEDS_PER_SEGMENT = 36;

    /** Total strip length in meters */
    public static final int STRIP_LENGTH_METERS = 1;

    /** Total number of logical addressable segments */
    // 20 segments per meter
    public static final int LOGICAL_LED_COUNT = SEGMENTS_PER_METER * STRIP_LENGTH_METERS; // TODO Adjust as needed
  }

}
