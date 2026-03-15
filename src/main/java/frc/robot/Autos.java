package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import swervelib.SwerveInputStream;

public class Autos {

        private final AutoFactory m_autoFactory;

        private final SwerveSubsystem m_swerveSubsystem;
        private final IntakeRollerSubsystem m_intakeRollerSubsystem;
        private final LinearIntakeSubsystem m_linearIntakeSubsystem;
        private final ShooterSubsystem m_shooterSubsystem;
        private final IndexerSubsystem m_indexerSubsystem;
        private final HopperSubsystem m_hopperSubsystem;

        private final DoubleSupplier m_autoAimHeadingX;
        private final DoubleSupplier m_autoAimHeadingY;

        public final SwerveInputStream stationaryAutoAim;

        public Autos(
                        AutoFactory autoFactory,
                        IntakeRollerSubsystem intakeRollerSubsystem,
                        LinearIntakeSubsystem linearIntakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        IndexerSubsystem indexerSubsystem,
                        HopperSubsystem hopperSubsystem,
                        SwerveSubsystem swerveSubsystem,
                        DoubleSupplier autoAimHeadingX,
                        DoubleSupplier autoAimHeadingY) {
                m_autoFactory = autoFactory;
                m_intakeRollerSubsystem = intakeRollerSubsystem;
                m_linearIntakeSubsystem = linearIntakeSubsystem;
                m_shooterSubsystem = shooterSubsystem;
                m_indexerSubsystem = indexerSubsystem;
                m_hopperSubsystem = hopperSubsystem;
                m_swerveSubsystem = swerveSubsystem;

                this.m_autoAimHeadingX = autoAimHeadingX;
                this.m_autoAimHeadingY = autoAimHeadingY;

                stationaryAutoAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                () -> 0.0,
                                () -> 0.0)
                                .deadband(OperatorConstants.DEADBAND)
                                .scaleTranslation(1.0)
                                .allianceRelativeControl(true)
                                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                                .headingWhile(true)
                                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
        }

        public Command rightAuto() {
                SwerveInputStream driveBackWithAutoAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                () -> -0.4,
                                () -> 0.17)
                                .deadband(OperatorConstants.DEADBAND)
                                .scaleTranslation(1.0)
                                .allianceRelativeControl(true)
                                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                                .headingWhile(true)
                                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
                ;

                Command driveBackWithAutoAimCmd = m_swerveSubsystem.driveFieldOriented(driveBackWithAutoAim);

                return Commands.sequence(
                                m_autoFactory.resetOdometry("RightAuto_1"),
                                m_autoFactory.trajectoryCmd("RightAuto_1"),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("RightAuto_2"),
                                                m_hopperSubsystem.expand(),
                                                // Note: Intake rollers never turn off in this auto
                                                m_intakeRollerSubsystem.intake(),
                                                m_linearIntakeSubsystem.extend()),
                                m_autoFactory.trajectoryCmd("RightAuto_3"),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("TrenchRightToAlliance"),
                                                m_linearIntakeSubsystem.retract()),
                                Commands.deadline(
                                                Commands.waitSeconds(2),
                                                m_swerveSubsystem.stop(),
                                                m_indexerSubsystem.run(),
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSubsystem.aimAndShoot(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true),
                                                                m_swerveSubsystem::isAutoAimOnTarget)),
                                // After shooting partially, retract hopper and linear intake while continuing
                                // to shoot
                                Commands.deadline(
                                                Commands.waitSeconds(1),
                                                m_indexerSubsystem.run(),
                                                m_hopperSubsystem.retract(),
                                                m_linearIntakeSubsystem.fullyRetract(),
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSubsystem.aimAndShoot(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true),
                                                                m_swerveSubsystem::isAutoAimOnTarget)),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("RightAuto_4"),
                                                m_indexerSubsystem.stop(),
                                                m_shooterSubsystem.stopShooting()),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("RightAuto_5"),
                                                m_hopperSubsystem.expand(),
                                                m_linearIntakeSubsystem.extend()),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("TrenchRightToAlliance"),
                                                m_linearIntakeSubsystem.retract()),
                                Commands.deadline(
                                                Commands.waitSeconds(3),
                                                driveBackWithAutoAimCmd,
                                                m_indexerSubsystem.run(),
                                                m_linearIntakeSubsystem.extend(),
                                                m_shooterSubsystem.aimAndShoot(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true),
                                                                m_swerveSubsystem::isAutoAimOnTarget)),
                                // After shooting partially, retract hopper and linear intake while continuing
                                // to shoot
                                Commands.deadline(
                                                Commands.waitSeconds(2),
                                                driveBackWithAutoAimCmd,
                                                m_indexerSubsystem.run(),
                                                m_hopperSubsystem.retract(),
                                                m_shooterSubsystem.aimAndShoot(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true),
                                                                m_swerveSubsystem::isAutoAimOnTarget)));
        }

        public Command shootPreloadAuto() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("ShootPreloadAuto"),
                                m_autoFactory.trajectoryCmd("ShootPreloadAuto"),
                                Commands.waitSeconds(3),
                                m_swerveSubsystem.stop(),
                                Commands.parallel(
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_indexerSubsystem.run(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true))));
        }

        public Command depotIntakeAuto() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("ToDepot"),
                                m_autoFactory.trajectoryCmd("ToDepot"),
                                Commands.deadline(
                                                Commands.waitSeconds(5),
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_indexerSubsystem.run(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true))),
                                Commands.deadline(
                                                Commands.waitSeconds(2.5),
                                                m_autoFactory.trajectoryCmd("IntakeDepot"),
                                                m_shooterSubsystem.stopShooting(),
                                                m_linearIntakeSubsystem.extend()),
                                Commands.waitSeconds(1),
                                Commands.deadline(
                                                Commands.waitSeconds(2.5),
                                                m_autoFactory.trajectoryCmd("ExitDepot"),
                                                m_linearIntakeSubsystem.retract()
                                                                .andThen(Commands.parallel(
                                                                                m_intakeRollerSubsystem.stop(),
                                                                                m_indexerSubsystem.stop()))),
                                Commands.parallel(
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_indexerSubsystem.run(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true))));
        }

        public Command depotOnlyAuto() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("ToDepot"),
                                m_autoFactory.trajectoryCmd("ToDepot"),
                                Commands.deadline(
                                                Commands.waitSeconds(2.5),
                                                m_autoFactory.trajectoryCmd("IntakeDepot"),
                                                m_linearIntakeSubsystem.extend(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_indexerSubsystem.run()),
                                Commands.waitSeconds(1),
                                Commands.deadline(
                                                Commands.waitSeconds(2.5),
                                                m_autoFactory.trajectoryCmd("ExitDepot"),
                                                m_linearIntakeSubsystem.retract()
                                                                .andThen(m_intakeRollerSubsystem.stop()),
                                                m_indexerSubsystem.stop()),
                                Commands.parallel(
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_indexerSubsystem.run(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true))));
        }

        public Command neutralAutoRight() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("RightAuto_1"),
                                m_autoFactory.trajectoryCmd("RightAuto_1"),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("RightAuto_2"),
                                                m_linearIntakeSubsystem.extend(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_indexerSubsystem.run()),
                                Commands.deadline(
                                                m_autoFactory.trajectoryCmd("Neutral1"),
                                                m_linearIntakeSubsystem.retract()
                                                                .andThen(Commands.parallel(
                                                                                m_intakeRollerSubsystem.stop(),
                                                                                m_indexerSubsystem.stop()))),
                                Commands.deadline(
                                                Commands.waitSeconds(10),
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true)))

                );
        }

        // public AutoRoutine sweepAutoRoutine() {
        //         AutoRoutine routine = m_autoFactory.newRoutine("SweepRoutine");

        //         AutoTrajectory towardsCenterLine = routine.trajectory("LeftPreload");
        //         AutoTrajectory towardsNeutralZone = routine.trajectory("LeftAuto_1");
        //         AutoTrajectory sweep = routine.trajectory("LeftPreload");
        //         AutoTrajectory towardsNeutralZone = routine.trajectory("LeftAuto_1");
        //         return null;
        // }

        public Command preLoadAndSweep() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("LeftPreload"),
                                m_autoFactory.trajectoryCmd("LeftPreload"),
                                Commands.deadline(
                                                Commands.waitSeconds(5),
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true))),
                                neutralAutoLeft());
        }

        // public Command neutralTest() {
        // return Commands.sequence(
        // m_autoFactory.resetOdometry("LeftAuto_1"),
        // m_autoFactory.trajectoryCmd("LeftAuto_1"),
        // Commands.deadline(
        // m_autoFactory.trajectoryCmd("LeftAuto_2"),
        // new InstantCommand(() -> SignalLogger.start())),
        // Commands.deadline(
        // m_autoFactory.trajectoryCmd("Neutral2"),
        // new InstantCommand(() -> SignalLogger.stop())),
        // Commands.parallel(
        // m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
        // m_shooterSubsystem.aimAndShootIgnoreCheck(
        // () -> m_swerveSubsystem.getDistanceToTarget(true))));
        // }

        public Command sweep(){
                return Commands.sequence(
                        m_autoFactory.resetOdometry("LeftAuto_1"),
                        m_autoFactory.trajectoryCmd("LeftAuto_1"),
                        m_autoFactory.trajectoryCmd("sweep")
                );
        }

        public Command neutralAutoLeft() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("LeftAuto_1"),
                                m_autoFactory.trajectoryCmd("LeftAuto_1"),
                                Commands.sequence(
                                                m_linearIntakeSubsystem.extend().andThen(
                                                        Commands.parallel(
                                                                m_intakeRollerSubsystem.intake(),
                                                                 m_indexerSubsystem.run()
                                                        )),
                                                 m_autoFactory.trajectoryCmd("LeftAuto_2")
                                                 )
                                                .withTimeout(5),
                                Commands.sequence(
                                                m_linearIntakeSubsystem.retract().andThen(
                                                        Commands.parallel(
                                                                m_intakeRollerSubsystem.stop(),
                                                                m_indexerSubsystem.stop())),
                                                 m_autoFactory.trajectoryCmd("Neutral2"))
                                                .withTimeout(5)
                                                .handleInterrupt(() -> Commands.none()),
                                Commands.parallel(
                                                m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                                                m_intakeRollerSubsystem.intake(),
                                                m_indexerSubsystem.run(),
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSubsystem.getDistanceToTarget(true)))

                );
        }
}
