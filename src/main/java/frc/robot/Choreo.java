package frc.robot;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.climb.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import swervelib.SwerveInputStream;

public class Choreo {

    private final AutoFactory m_autoFactory;

    private final RobotContainer m_robotContainer;

    private final IndexerSubsystem m_indexerSubsystem;
    private final IntakeRollerSubsystem m_intakeRollerSubsystem;
    private final LinearIntakeSubsystem m_linearIntakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final DoubleSupplier m_autoAimHeadingX;
    private final DoubleSupplier m_autoAimHeadingY;

    public final SwerveInputStream stationaryAutoAim;
    public final SwerveInputStream aimTowardsRed;
    public final SwerveInputStream aimTowardsBlue;

    public final SwerveInputStream backUpAndAimRed;
    public final SwerveInputStream backUpAndAimBlue;

    public Choreo(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;

        m_autoFactory = robotContainer.m_autoFactory;

        m_indexerSubsystem = robotContainer.m_indexerSubsystem;
        m_intakeRollerSubsystem = robotContainer.m_intakeRollerSubsystem;
        m_linearIntakeSubsystem = robotContainer.m_linearIntakeSubsystem;
        m_shooterSubsystem = robotContainer.m_shooterSubsystem;
        m_swerveSubsystem = robotContainer.m_swerveSubsystem;
        m_elevatorSubsystem = robotContainer.m_elevatorSubsystem;

        m_autoAimHeadingX = robotContainer.autoAimHeadingX();
        m_autoAimHeadingY = robotContainer.autoAimHeadingY();

        stationaryAutoAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                () -> 0.0,
                () -> 0.0)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true)
                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                .headingWhile(true)
                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

        aimTowardsRed = stationaryAutoAim.copy()
                .withControllerHeadingAxis(() -> 0.0, () -> -1.0);

        aimTowardsBlue = stationaryAutoAim.copy()
                .withControllerHeadingAxis(() -> 0.0, () -> 1.0);

        backUpAndAimRed = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                () -> -0.25,
                () -> 0.0)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true)
                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                .headingWhile(true)
                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

        backUpAndAimBlue = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                () -> 0.25,
                () -> 0.0)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true)
                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                .headingWhile(true)
                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
    }

    public Command shootPreloadAuto() {
        return Commands.sequence(
                // m_autoFactory.resetOdometry("ShootPreloadAuto"),
                // m_autoFactory.trajectoryCmd("ShootPreloadAuto"),
                // new InstantCommand(
                // () -> m_swerveSubsystem.drive(new Translation2d(-0.25, 0.0), 0, false)),
                Commands.waitSeconds(3).deadlineFor(new ConditionalCommand(
                        m_swerveSubsystem.driveFieldOriented(backUpAndAimRed),
                        m_swerveSubsystem.driveFieldOriented(backUpAndAimBlue),
                        m_swerveSubsystem::isRedAlliance)),
                m_swerveSubsystem.stop(),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake(),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))));
    }

    private Command rightNeutralAutoFirstSweep() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("RightAuto1"),
                m_autoFactory.trajectoryCmd("RightAuto1").deadlineFor(
                        m_intakeRollerSubsystem.intake(),
                        m_indexerSubsystem.run()),
                m_swerveSubsystem.stop());
    }

    private Command leftNeutralAutoFirstSweep() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("LeftAuto1"),
                m_autoFactory.trajectoryCmd("LeftAuto1").deadlineFor(
                        m_intakeRollerSubsystem.intake(),
                        m_indexerSubsystem.run()),
                m_swerveSubsystem.stop());
    }

    public Command leftNeutralAutoSweepOnce() {
        return Commands.sequence(
                leftNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("LeftAuto2a"),
                m_swerveSubsystem.stop(),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_linearIntakeSubsystem.shuffle()));
    }

    public Command rightNeutralAutoSweepOnce() {
        return Commands.sequence(
                rightNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("RightAuto2a"),
                m_swerveSubsystem.stop(),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_linearIntakeSubsystem.shuffle()));
    }

    public Command leftNeutralAutoSweepTwice() {
        return Commands.sequence(
                leftNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("LeftAuto2a"),
                m_swerveSubsystem.stop(),
                Commands.waitSeconds(8).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_linearIntakeSubsystem.shuffle()),
                m_autoFactory.trajectoryCmd("LeftAuto3a").deadlineFor(
                        m_shooterSubsystem.stopShooting(),
                        m_linearIntakeSubsystem.extend()),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_linearIntakeSubsystem.shuffle(),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake()));
    }

    public Command rightNeutralAutoSweepTwice() {
        return Commands.sequence(
                rightNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("RightAuto2a"),
                m_swerveSubsystem.stop(),
                Commands.waitSeconds(8).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_linearIntakeSubsystem.shuffle()),
                m_autoFactory.trajectoryCmd("RightAuto3a").deadlineFor(
                        m_shooterSubsystem.stopShooting(),
                        m_linearIntakeSubsystem.extend()),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake(),
                        m_linearIntakeSubsystem.shuffle()));
    }

    public Command rightNeutralAutoThenClimb() {
        return Commands.sequence(
                rightNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("RightAuto2b"),
                m_swerveSubsystem.stop(),
                // TODO: Tune timing of these waits and commands
                Commands.waitSeconds(4.5).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))),
                Commands.waitSeconds(1.0).deadlineFor(
                        new ConditionalCommand(
                                m_swerveSubsystem.driveFieldOriented(aimTowardsBlue),
                                m_swerveSubsystem.driveFieldOriented(aimTowardsRed),
                                m_swerveSubsystem::isRedAlliance)),
                m_autoFactory.trajectoryCmd("RightAuto3b").deadlineFor(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSubsystem.stop(),
                        m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                        m_linearIntakeSubsystem.retract()),
                m_swerveSubsystem.stop(),
                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
    }

    public Command leftNeutralAutoThenDepot() {
        return Commands.sequence(
                leftNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("LeftAuto2c"),
                m_autoFactory.trajectoryCmd("LeftAuto3c").deadlineFor(
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))));
    }

    public Command leftNeutralAutoThenClimb() {
        return Commands.sequence(
                leftNeutralAutoFirstSweep(),
                m_autoFactory.trajectoryCmd("LeftAuto2b"),
                m_swerveSubsystem.stop(),
                // TODO: Tune timing of these waits and commands
                Commands.waitSeconds(4.5).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))),
                Commands.waitSeconds(1.0).deadlineFor(
                        new ConditionalCommand(
                                m_swerveSubsystem.driveFieldOriented(aimTowardsBlue),
                                m_swerveSubsystem.driveFieldOriented(aimTowardsRed),
                                m_swerveSubsystem::isRedAlliance)),
                m_autoFactory.trajectoryCmd("LeftAuto3b").deadlineFor(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSubsystem.stop(),
                        m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                        m_linearIntakeSubsystem.retract()),
                m_swerveSubsystem.stop(),
                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
    }

    public Command shootPreloadAndClimbLeft() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("BackUpCenterLeft"),
                m_autoFactory.trajectoryCmd("BackUpCenterLeft"),
                Commands.waitSeconds(4.5).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))),
                m_autoFactory.trajectoryCmd("CenterToClimbLeft"),
                m_swerveSubsystem.stop(),
                m_autoFactory.trajectoryCmd("LeftAuto3b").deadlineFor(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSubsystem.stop(),
                        m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                        m_linearIntakeSubsystem.retract()),
                m_swerveSubsystem.stop(),
                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
    }

    public Command shootPreloadAndClimbRight() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("BackUpCenterRight"),
                m_autoFactory.trajectoryCmd("BackUpCenterRight"),
                Commands.waitSeconds(4.5).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))),
                m_autoFactory.trajectoryCmd("CenterToClimbRight"),
                m_swerveSubsystem.stop(),
                m_autoFactory.trajectoryCmd("RightAuto3b").deadlineFor(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSubsystem.stop(),
                        m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                        m_linearIntakeSubsystem.retract()),
                m_swerveSubsystem.stop(),
                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
    }
}