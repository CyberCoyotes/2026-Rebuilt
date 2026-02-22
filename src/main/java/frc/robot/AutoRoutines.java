package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import frc.robot.commands.AutoAlignAndShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final VisionSubsystem m_vision;

    public AutoRoutines(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_shooter = shooter;
        m_indexer = indexer;
        m_vision = vision;
    }

    public AutoRoutine blueRight_CenterRun() {
        final AutoRoutine routine = m_factory.newRoutine("BR_CenterRun");
        final AutoTrajectory path = routine.trajectory("BR_CenterRun");

        routine.active().onTrue(
            path.resetOdometry()
                .andThen(path.cmd().alongWith(
                    m_shooter.runOnce(m_shooter::closeShot)
                ))
                .andThen(new AutoAlignAndShootCommand(
                    m_drivetrain, m_shooter, m_vision, m_indexer, 5.0
                ))
        );

        path.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        return routine;
    }

    public AutoRoutine blueRight_CenterThenSwitch() {
        final AutoRoutine routine = m_factory.newRoutine("BR_CenterThenSwitch");
        final AutoTrajectory centerRun = routine.trajectory("BR_CenterRun");
        final AutoTrajectory switchRun = routine.trajectory("BR_2_SwitchRun");

        routine.active().onTrue(
            centerRun.resetOdometry()
                .andThen(centerRun.cmd().alongWith(
                    m_shooter.runOnce(m_shooter::closeShot)
                ))
                .andThen(new AutoAlignAndShootCommand(
                    m_drivetrain, m_shooter, m_vision, m_indexer, 5.0
                ))
                .andThen(switchRun.cmd())
        );

        centerRun.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        centerRun.atTime("StopIntake").onTrue(m_intake.stopFuel());
        switchRun.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        switchRun.atTime("StopIntake").onTrue(m_intake.stopFuel());

        return routine;
    }

    public AutoRoutine blueRight_CenterThenClimb() {
        final AutoRoutine routine = m_factory.newRoutine("BR_CenterThenClimb");
        final AutoTrajectory centerRun = routine.trajectory("BR_CenterRun");
        final AutoTrajectory climb = routine.trajectory("BR_2_Climb");

        routine.active().onTrue(
            centerRun.resetOdometry()
                .andThen(centerRun.cmd().alongWith(
                    m_shooter.runOnce(m_shooter::closeShot)
                ))
                .andThen(new AutoAlignAndShootCommand(
                    m_drivetrain, m_shooter, m_vision, m_indexer, 5.0
                ))
                .andThen(climb.cmd())
        );

        centerRun.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        centerRun.atTime("StopIntake").onTrue(m_intake.stopFuel());

        // TODO: bind these to your climber subsystem once it's implemented
        climb.atTime("RetractIntake").onTrue(m_intake.stopFuel());
        // climb.atTime("PauseExtendClimber").onTrue(climber.pauseExtend());
        // climb.atTime("RetractClimber").onTrue(climber.retract());

        return routine;
    }
}