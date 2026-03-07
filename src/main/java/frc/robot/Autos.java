package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    }

    public Command rightAuto() {

        SwerveInputStream stationaryAutoAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                () -> 0.0,
                () -> 0.0)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true)
                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                .headingWhile(true)
                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

        Command stationaryAutoAimCmd = m_swerveSubsystem.driveFieldOriented(stationaryAutoAim);

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
                        stationaryAutoAimCmd,
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
                        stationaryAutoAimCmd,
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
                m_autoFactory.trajectoryCmd("ShootPreloadAuto"),
                Commands.waitSeconds(3),
                Commands.parallel(
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake(),
                        m_shooterSubsystem.shootWithHoodAngle(Degrees.of(7))));
    }
}
