package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

final class LeftAutoRoutines {
    private final AutoRoutineContext ctx;

    LeftAutoRoutines(AutoRoutineContext ctx) {
        this.ctx = ctx;
    }

    public AutoRoutine LtTrench_Ramp_Single() {
        final AutoRoutine routine = ctx.factory.newRoutine("Left x1 Trench-Ramp");
        final AutoTrajectory ltTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
        final AutoTrajectory ltRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);
        final AutoTrajectory ltShootRamp = routine.trajectory("LtShootRamp", 0);

        routine.active().onTrue(
                Commands.sequence(
                        ltTrench_Middle.resetOdometry(),
                        Commands.deadline(ltTrench_Middle.cmd(), ctx.intakeTimed()),
                        ltRampMiddle_Alliance.cmd(),
                        ltShootRamp.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine LtTrench_Ramp_Double() {
        final AutoRoutine routine = ctx.factory.newRoutine("Left x2 Trench-Ramp");
        final AutoTrajectory ltTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
        final AutoTrajectory ltRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);
        final AutoTrajectory ltShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);
        final AutoTrajectory ltTrench_Middle_2 = routine.trajectory("LtTrench_Middle", 0);
        final AutoTrajectory ltRampMiddle_Alliance_2 = routine.trajectory("LtRampMiddle_Alliance", 0);

        routine.active().onTrue(
                Commands.sequence(
                        ltTrench_Middle.resetOdometry(),
                        Commands.deadline(ltTrench_Middle.cmd(), ctx.intakeTimed()),
                        ltRampMiddle_Alliance.cmd(),
                        ltShootRamp_Trench.cmd(),
                        ctx.alignAndShoot(),
                        ltShootRamp_Trench.cmd(),
                        Commands.deadline(ltTrench_Middle_2.cmd(), ctx.intakeTimed()),
                        ltRampMiddle_Alliance_2.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine LtTrench_Ramp_HubSweep() {
        final AutoRoutine routine = ctx.factory.newRoutine("Left Trench-Ramp-HubSweep");
        final AutoTrajectory ltTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
        final AutoTrajectory ltRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);
        final AutoTrajectory ltShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);
        final AutoTrajectory ltTrench_HubSweep = routine.trajectory("LtTrench_HubSweep", 0);
        final AutoTrajectory ltHub_Ramp = routine.trajectory("LtHub_Ramp", 0);
        final AutoTrajectory ltRampMiddle_Alliance_2 = routine.trajectory("LtRampMiddle_Alliance", 0);

        routine.active().onTrue(
                Commands.sequence(
                        ltTrench_Middle.resetOdometry(),
                        Commands.deadline(ltTrench_Middle.cmd(), ctx.intakeTimed()),
                        ltRampMiddle_Alliance.cmd(),
                        ltShootRamp_Trench.cmd(),
                        ctx.alignAndShoot(),
                        ltShootRamp_Trench.cmd(),
                        Commands.deadline(ltTrench_HubSweep.cmd(), ctx.intakeTimed()),
                        ltHub_Ramp.cmd(),
                        ltRampMiddle_Alliance_2.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine LtTrench_Ramp_Sweep_Purge() {
        final AutoRoutine routine = ctx.factory.newRoutine("Left Trench-Ramp-Sweep");
        final AutoTrajectory ltTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
        final AutoTrajectory ltRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);
        final AutoTrajectory ltShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);
        final AutoTrajectory ltTrench_HubSweep = routine.trajectory("LtTrench_HubSweep", 0);
        final AutoTrajectory ltHub_Purge = routine.trajectory("LtHub_Purge", 0);

        routine.active().onTrue(
                Commands.sequence(
                        ltTrench_Middle.resetOdometry(),
                        Commands.deadline(ltTrench_Middle.cmd(), ctx.intakeTimed()),
                        ltRampMiddle_Alliance.cmd(),
                        ltShootRamp_Trench.cmd(),
                        ctx.alignAndShoot(),
                        ltShootRamp_Trench.cmd(),
                        Commands.deadline(ltTrench_HubSweep.cmd(), ctx.intakeTimed()),
                        ltHub_Purge.cmd(),
                        ctx.purgeFuel()));

        return routine;
    }

    public AutoRoutine LtTrench_Ramp_Sweep_AngryPurge() {
        return LtTrench_Ramp_Sweep_Purge();
    }
}
