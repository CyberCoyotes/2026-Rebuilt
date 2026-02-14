package frc.robot.subsystems.intake;

public class IntakeConstants {

     final static int INTAKE_SENSOR_ID = 12345;
     final static int INTAKE_THRESHOLD = 1000; //mm, around four inches

     final static int INDEXER_SENSOR_ID = 234;
     final static int INDEXER_THRESHOLD = 67; //mm

     final static double JAM_CURRENT_THRESHOLD = 20.0; // current should be under this
     final static double JAM_VELOCITY_THRESHOLD = 0.5; // velocity should be over this

     final static double SLIDE_EXTENDED_POSITION = 1.85;
     final static double SLIDE_RESTING_POSITION = 0;
     final static double ROTATOR_RUNNING_VELOCITY = 0.5;

     final static double ROTATOR_MAX_VELOCITY = 1;
    
}
