package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {
  private Constants() {}

  /** CTRE CAN bus name (empty string means "rio") */
  public static final CANBus kCANBus = new CANBus("rio");

  // =========================================================
  // Drive / Swerve
  // =========================================================
  // Here for reference only! CTRE Swerve drive will generate it's own
  public static final class Drive {
    private Drive() {}

    // Front Left
    // public static final int FL_DRIVE_MOTOR_ID = 1;
    // public static final int FL_STEER_MOTOR_ID = 2;

    // Front Right
    // public static final int FR_DRIVE_MOTOR_ID = 3;
    // public static final int FR_STEER_MOTOR_ID = 4;

    // Back Left
    // public static final int BL_DRIVE_MOTOR_ID = 5;
    // public static final int BL_STEER_MOTOR_ID = 6;

    // Back Right
    // public static final int BR_DRIVE_MOTOR_ID = 7;
    // public static final int BR_STEER_MOTOR_ID = 8;

    // Navigation
    // Pigeon
    
    // Feedback
    // CANDLE

  }

  // =========================================================
  // Intake
  // =========================================================
  public static final class Intake {
    private Intake() {}

    public static final int INTAKE_ROTATOR_ID = 20;
    public static final int INTAKE_SLIDE_ID = 21;
  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {
    private Indexer() {}

    public static final int FEEDER_MOTOR_ID = 22;
    public static final int FLOOR_MOTOR_ID = 23;
    public static final int FEEDER_TOF_ID = 30;
    public static final double FUEL_DETECTION_THRESHOLD = 0;


  }

  // =========================================================
  // Shooter
  // =========================================================
  public static final class Shooter {
    private Shooter() {}

    // Main flywheels (Falcon 500 / TalonFX)
    public static final int FLYWHEEL_A_MOTOR_ID = 24;
    public static final int FLYWHEEL_B_MOTOR_ID = 25;
    public static final int FLYWHEEL_C_MOTOR_ID = 26;

    /**
     * Hood movement motor (Minion) — ONLY keep this as a CAN ID if the motor controller is on CAN.
     * If this is PWM (SparkMAX PWM / Talon SRX PWM / etc.), move this to a PWM constants section.
     */
    public static final int HOOD_MOTOR_ID = 27;

    // Counter wheel (Kraken / TalonFX)
    public static final int COUNTER_WHEEL_MOTOR_ID = 28;
  }

  // =========================================================
  // Climber
  // =========================================================
  public static final class Climber {
    private Climber() {}

    public static final int CLIMB_EXTENSION_MOTOR_ID = 29;
    /**
     * Hook retraction motor (Minion) — ONLY keep this as a CAN ID if the motor controller is on CAN.
     * If this is PWM, move to a PWM constants section.
     */
    public static final int HOOK_RETRACT_MOTOR_ID = 30;
  }

  // =========================================================
  // Vision / Limelight
  // =========================================================
  public static final class Vision {
    private Vision() {}

    // Camera configuration
    public static final String LIMELIGHT_NAME = "limelight";

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
  // Optional: PWM (if Minion motors are NOT on CAN)
  // =========================================================
  // public static final class PWM {
  //   private PWM() {}
  //
  //   // Example:
  //   // public static final int SHOOTER_HOOD_PWM_CHANNEL = 0;
  //   // public static final int CLIMBER_HOOK_PWM_CHANNEL = 1;
  // }
}