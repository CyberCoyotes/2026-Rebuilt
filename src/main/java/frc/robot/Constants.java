package frc.robot;

public final class Constants {
  private Constants() {}

  /** CTRE CAN bus name (empty string means "rio") */
  public static final String kCANBus = "rio";

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

    public static final int INTAKE_ROLLER_MOTOR_ID = 20;
    public static final int INTAKE_DEPLOY_RACK_PINION_MOTOR_ID = 21;
    public static final int INTAKE_TOF_SENSOR_ID = 9000; //TODO: filler ID
    public static final int INTAKE_TOF_THRESHOLD = 100; //in milimeters, around four inches
  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {
    private Indexer() {}

    public static final int ELEVATOR_TO_SHOOTER_MOTOR_ID = 22;
    public static final int ACTIVE_FLOOR_MOTOR_ID = 23;
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