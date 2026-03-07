package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;

public class Autos {

    private final AutoFactory m_autoFactory;

    private final SwerveSubsystem m_swerveSubsystem;
    private final IntakeRollerSubsystem m_intakeRollerSubsystem;
    private final LinearIntakeSubsystem m_linearIntakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IndexerSubsystem m_indexerSubsystem;
    private final HopperSubsystem m_hopperSubsystem;

    public Autos(
            AutoFactory autoFactory,
            IntakeRollerSubsystem intakeRollerSubsystem,
            LinearIntakeSubsystem linearIntakeSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            HopperSubsystem hopperSubsystem,
            SwerveSubsystem swerveSubsystem) {
        m_autoFactory = autoFactory;
        m_intakeRollerSubsystem = intakeRollerSubsystem;
        m_linearIntakeSubsystem = linearIntakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        m_hopperSubsystem = hopperSubsystem;
        m_swerveSubsystem = swerveSubsystem;
    }

    public Command rightAuto() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("RightAuto_1"),
                m_autoFactory.trajectoryCmd("RightAuto_1"),
                Commands.deadline(
                        m_autoFactory.trajectoryCmd("RightAuto_2"),
                        m_intakeRollerSubsystem.intake(),
                        m_linearIntakeSubsystem.extend()),
                m_autoFactory.trajectoryCmd("RightAuto_3"),
                Commands.deadline(
                        m_autoFactory.trajectoryCmd("TrenchRightToAlliance"),
                        m_linearIntakeSubsystem.retract())
        // m_shooterSubsystem.aimAndShoot(() ->
        // m_swerveSubsystem.getDistanceToTarget(true),
        // m_swerveSubsystem::isAutoAimOnTarget)
        );
    }
}
