package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FuelCommands;

final class CenterAutoRoutines {
    private final AutoRoutineContext ctx;

    CenterAutoRoutines(AutoRoutineContext ctx) {
        this.ctx = ctx;
    }

    public AutoRoutine Center() {
        final AutoRoutine routine = ctx.factory.newRoutine("Center");
        final AutoTrajectory center = routine.trajectory("Center", 0);

        routine.active().onTrue(
                Commands.sequence(
                        center.resetOdometry(),
                        center.cmd(),
                        ctx.drivetrain.stop().withTimeout(10.0)));

        center.atTime("Shoot").onTrue(FuelCommands.Auto.shootClose(ctx.shooter, ctx.indexer, Constants.Auto.SHOOT_TIMEOUT));
        return routine;
    }

    public final class Test {
        public AutoRoutine Full_LtTrench_Mid_Trench() {
            final AutoRoutine routine = ctx.factory.newRoutine("FULL Lt Trench-Mid-Trench");
            final AutoTrajectory ltTrench_Mid_Trench = routine.trajectory("Full_LtTrench_Mid_Trench", 0);

            routine.active().onTrue(Commands.sequence(ltTrench_Mid_Trench.resetOdometry(), ltTrench_Mid_Trench.cmd()));
            ltTrench_Mid_Trench.atTime("Intake").onTrue(ctx.intakeTimed());
            ltTrench_Mid_Trench.atTime("Shoot").onTrue(ctx.alignAndShoot());
            ltTrench_Mid_Trench.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(ctx.intake, ctx.indexer));
            return routine;
        }

        public AutoRoutine MidDepot() {
            final AutoRoutine routine = ctx.factory.newRoutine("MidDepot");
            final AutoTrajectory center_MidDepot = routine.trajectory("Center_MidDepot", 0);

            routine.active().onTrue(Commands.sequence(center_MidDepot.resetOdometry(), center_MidDepot.cmd()));
            center_MidDepot.atTime("Intake").onTrue(ctx.intake.intakeFuelTimer(8, Constants.Auto.INTAKE_DELAY));
            center_MidDepot.atTime("Shoot").onTrue(FuelCommands.Auto.shootFar(ctx.shooter, ctx.indexer, Constants.Auto.SHOOT_TIMEOUT));
            center_MidDepot.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(ctx.intake, ctx.indexer));
            return routine;
        }

        public AutoRoutine RtTrench_RtMid_RtTrench() {
            final AutoRoutine routine = ctx.factory.newRoutine("RtTrench_RtMid_RtTrench");
            final AutoTrajectory rtTrench_RtMid_RtTrench = routine.trajectory("RtTrench_RtMid_RtTrench", 0);

            routine.active().onTrue(Commands.sequence(rtTrench_RtMid_RtTrench.resetOdometry(), rtTrench_RtMid_RtTrench.cmd()));
            rtTrench_RtMid_RtTrench.atTime("Intake").onTrue(ctx.intakeTimed());
            rtTrench_RtMid_RtTrench.atTime("Shoot").onTrue(ctx.alignAndShoot());
            rtTrench_RtMid_RtTrench.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(ctx.intake, ctx.indexer));
            return routine;
        }

        public AutoRoutine Lob() {
            final AutoRoutine routine = ctx.factory.newRoutine("Lob");
            final AutoTrajectory lob = routine.trajectory("Lob", 0);

            routine.active().onTrue(
                    Commands.sequence(
                            lob.resetOdometry(),
                            lob.cmd(),
                            ctx.drivetrain.stop().withTimeout(Constants.Auto.DRIVE_WAIT)));
            return routine;
        }

        public AutoRoutine visionTest() {
            final AutoRoutine routine = ctx.factory.newRoutine("VisionTest");
            final AutoTrajectory experimental = routine.trajectory("VisionTest", 0);

            routine.active().onTrue(Commands.sequence(experimental.resetOdometry(), experimental.cmd()));
            experimental.atTime("Shoot").onTrue(ctx.alignAndShoot());
            return routine;
        }

        public AutoRoutine FM() {
            final AutoRoutine routine = ctx.factory.newRoutine("FourMeters");
            final AutoTrajectory fm = routine.trajectory("FourMeters", 0);
            routine.active().onTrue(Commands.sequence(fm.resetOdometry(), fm.cmd()));
            return routine;
        }

        public AutoRoutine STAtoL() {
            final AutoRoutine routine = ctx.factory.newRoutine("ST-A");
            final AutoTrajectory sta = routine.trajectory("ST-A", 0);
            final AutoTrajectory sta2 = routine.trajectory("ST-A", 1);
            final AutoTrajectory csl = routine.trajectory("CS1-L", 0);
            final AutoTrajectory csl2 = routine.trajectory("CS1-L", 1);

            routine.active().onTrue(
                    Commands.sequence(
                            sta.resetOdometry(),
                            sta.cmd(),
                            sta2.cmd(),
                            csl.cmd(),
                            csl2.cmd()));
            return routine;
        }
    }
}
