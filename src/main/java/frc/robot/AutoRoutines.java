package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoRoutines {
    private final RightAutoRoutines right;
    private final LeftAutoRoutines left;
    private final CenterAutoRoutines center;

    public AutoRoutines(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            ShooterSubsystem shooter) {
        final AutoRoutineContext context = new AutoRoutineContext(factory, drivetrain, indexer, intake, shooter);
        right = new RightAutoRoutines(context);
        left = new LeftAutoRoutines(context);
        center = new CenterAutoRoutines(context);
    }

    public AutoRoutine RtTrench_Ramp_Single() { return right.RtTrench_Ramp_Single(); }
    public AutoRoutine RtTrench_Ramp_Meep() { return right.RtTrench_Ramp_Meep(); }
    public AutoRoutine RtTrench_Ramp_Double() { return right.RtTrench_Ramp_Double(); }
    public AutoRoutine RtTrench_Ramp_HubSweep() { return right.RtTrench_Ramp_HubSweep(); }
    public AutoRoutine RtTrench_Ramp_Sweep_Purge() { return right.RtTrench_Ramp_Sweep_Purge(); }
    public AutoRoutine RtTrench_Ramp_Sweep_AngryPurge() { return right.RtTrench_Ramp_Sweep_AngryPurge(); }
    public AutoRoutine RtTrench_Ramp_AngryMeepMeep() { return right.RtTrench_Ramp_AngryMeepMeep(); }
    public AutoRoutine RtBulldozer() { return right.RtBulldozer(); }

    public AutoRoutine LtTrench_Ramp_Single() { return left.LtTrench_Ramp_Single(); }
    public AutoRoutine LtTrench_Ramp_Double() { return left.LtTrench_Ramp_Double(); }
    public AutoRoutine LtTrench_Ramp_HubSweep() { return left.LtTrench_Ramp_HubSweep(); }
    public AutoRoutine LtTrench_Ramp_Sweep_Purge() { return left.LtTrench_Ramp_Sweep_Purge(); }
    public AutoRoutine LtTrench_Ramp_Sweep_AngryPurge() { return left.LtTrench_Ramp_Sweep_AngryPurge(); }

    public AutoRoutine Center() { return center.Center(); }

    public class Test {
        private final CenterAutoRoutines.Test test = center.new Test();

        public AutoRoutine Full_LtTrench_Mid_Trench() { return test.Full_LtTrench_Mid_Trench(); }
        public AutoRoutine MidDepot() { return test.MidDepot(); }
        public AutoRoutine RtTrench_RtMid_RtTrench() { return test.RtTrench_RtMid_RtTrench(); }
        public AutoRoutine Lob() { return test.Lob(); }
        public AutoRoutine visionTest() { return test.visionTest(); }
        public AutoRoutine FM() { return test.FM(); }
        public AutoRoutine STAtoL() { return test.STAtoL(); }
    }
}
