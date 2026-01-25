package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {
  private Constants() {}

  /** CTRE CAN bus name (empty string means "rio") */
  public static final CANBus kCANBus = new CANBus("rio");

  // =========================================================
  // Drive / Swerve
  // =========================================================
  // RESERVE CAN IDs 1 to 10 for drivertrain; drive motors, pigeon 2, canivore.
  // Those IDs live in TunerConstants.java
  
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
    public static final int INTAKE_SLIDE_B__MOTOR_ID = 51;

    /** Time of Flight sensor - Playing With Fusion, confirms fuel presence */
    public static final int INTAKE_SENSOR_ID = 41;
  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {
    private Indexer() {}

    /** Indexer motor - Kraken X60 with TalonFX controller, feeds pieces to shooter */
    public static final int INDEXER_MOTOR_ID = 22;

    /** Conveyor motor - Kraken X60 with TalonFX controller, moves pieces along hopper */
    public static final int CONVEYOR_MOTOR_ID = 23;

    /** Time of Flight sensor - Playing With Fusion, detects fuel at indexer exit (optional) */
    public static final int INDEXER_TOF_ID = 42;

    /** Time of Flight sensor A - Playing With Fusion, hopper position A detection */
    public static final int HOPPER_TOP_A_TOF_ID = 43;

    /** Time of Flight sensor B - Playing With Fusion, hopper position B detection */
    public static final int HOPPER_TOP_B_TOF_ID = 44;

    /** Time of Flight sensor C - Playing With Fusion, hopper position C detection */
    public static final int HOPPER_TOP_C_TOF_ID = 45;
  }

  // =========================================================
  // Shooter
  // =========================================================
  public static final class Shooter {
    private Shooter() {}

    /** Flywheel A motor - Falcon 500 with TalonFX controller (leader) */
    public static final int FLYWHEEL_A_MOTOR_ID = 24;

    /** Flywheel B motor - Falcon 500 with TalonFX controller (follower of A) */
    public static final int FLYWHEEL_B_MOTOR_ID = 25;

    /** Flywheel C motor - Falcon 500 with TalonFX controller (follower of A) */
    public static final int FLYWHEEL_C_MOTOR_ID = 26;

    /** Hood motor - Minion with TalonFXS controller, adjusts shot angle */
    public static final int HOOD_MOTOR_ID = 27;

    /** Counter wheel motor - Kraken X44 with TalonFX controller */
    public static final int COUNTER_WHEEL_MOTOR_ID = 28;
  }

  // =========================================================
  // Climber
  // =========================================================
  public static final class Climber {
    private Climber() {}

    /** Climber motor A - Kraken X60 with TalonFX controller */
    public static final int CLIMB_A_MOTOR_ID = 29;

    /** Climber motor B - Kraken X60 with TalonFX controller */
    public static final int CLIMB_B_MOTOR_ID = 30;
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

}