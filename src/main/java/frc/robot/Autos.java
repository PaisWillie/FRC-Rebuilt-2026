package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants.Position;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.Zone;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import swervelib.SwerveInputStream;

public class Autos {

    private final RobotContainer m_robotContainer;

    private final IndexerSubsystem m_indexerSubsystem;
    private final IntakeRollerSubsystem m_intakeRollerSubsystem;
    private final LinearIntakeSubsystem m_linearIntakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    private final SwerveInputStream m_driveAngularVelocity;
    private final Command m_driveFieldOrientedAngularVelocity;
    private final Command m_driveFieldOrientedAutoAim;

    private final AutoFactory m_autoFactory;

    public Autos(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;

        m_indexerSubsystem = robotContainer.m_indexerSubsystem;
        m_intakeRollerSubsystem = robotContainer.m_intakeRollerSubsystem;
        m_linearIntakeSubsystem = robotContainer.m_linearIntakeSubsystem;
        m_shooterSubsystem = robotContainer.m_shooterSubsystem;
        m_swerveSubsystem = robotContainer.m_swerveSubsystem;

        m_driveAngularVelocity = m_robotContainer.driveAngularVelocity;

        m_driveFieldOrientedAngularVelocity = m_robotContainer.driveFieldOrientedAngularVelocity;
        m_driveFieldOrientedAutoAim = m_robotContainer.driveFieldOrientedAutoAim;

        m_autoFactory = robotContainer.autoFactory;

        m_autoFactory
                .bind("Intake", Commands.sequence(
                        m_linearIntakeSubsystem.extend(),
                        m_intakeRollerSubsystem.intake(),
                        m_indexerSubsystem.run()
                ))
                .bind("StopIntake", m_linearIntakeSubsystem.midpoint().andThen(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSubsystem.stop()
                ))
                .bind("Shoot", Commands.parallel(
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                m_swerveSubsystem::getDistanceToTarget),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake()
                ));

        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            StructArrayPublisher<Pose2d> waypointPositions = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Waypoint Positions", Pose2d.struct)
                    .publish();

            Position[] waypointSelection = {
                    Position.STARTING_LINE_RIGHT,
                    Position.NEUTRAL_RIGHT_1,
                    Position.NEUTRAL_RIGHT_2,
                    Position.NEUTRAL_RIGHT_3,
                    Position.ALLIANCE_RIGHT_1
            };

            waypointPositions.accept(
                    java.util.Arrays.stream(waypointSelection)
                            .map(AutoConstants.positionToPose::get)
                            // .map((position) ->
                            // PoseUtils.flipX(AutoConstants.positionToPose.get(position)))
                            .toArray(Pose2d[]::new));
        }
    }

    private Command driveToWaypoint(Pose2d waypoint) {
        return new InstantCommand(
                () -> {
                    m_swerveSubsystem.setDriveToWaypoint(waypoint);
                    SmartDashboard.putString("driveToWaypointCHECK", "WAYPOINT SET");
                })

                // Wait until the robot is within the specified tolerance of the waypoint
                .andThen(new InstantCommand(
                        () -> SmartDashboard.putString("driveToWaypointCHECK", "WAIT ONE SEC")))
                .andThen(Commands.waitSeconds(1.0))
                .andThen(new InstantCommand(
                        () -> SmartDashboard.putString("driveToWaypointCHECK", "CHECKING ISATWAYPOINT")))
                .andThen(Commands.waitUntil(
                        m_swerveSubsystem::isAtWaypoint))
                .andThen(new InstantCommand(() -> SmartDashboard.putString("driveToWaypointCHECK", "CHECK COMPLETE")));
    }

    private Command driveToWaypoint(Pose2d waypoint, Angle angleTolerance) {
        return new InstantCommand(
                () -> m_swerveSubsystem.setDriveToWaypoint(waypoint))

                // Wait until the robot is within the specified tolerance of the waypoint
                .andThen(Commands.waitUntil(
                        () -> m_swerveSubsystem.isAtWaypoint(
                                AutoConstants.DEFAULT_WAYPOINT_TOLERANCE, angleTolerance.in(Degrees))));
    }

    private Command driveToWaypoint(Position position) {
        return driveToWaypoint(AutoConstants.positionToPose.get(position));
    }

    private Command driveToWaypoint(Position position, Angle angleTolerance) {
        return driveToWaypoint(AutoConstants.positionToPose.get(position), angleTolerance);
    }

    // private Command driveToWaypoint(Position position, double
    // maxTranslationalVelocity) {
    // return Commands.sequence(
    // new InstantCommand(() -> updateLimits(maxTranslationalVelocity)),
    // driveToWaypoint(position));
    // }

    // private Command driveToWaypoint(Position position, double
    // maxTranslationalVelocity,
    // double maxTranslationalAcceleration) {
    // return Commands.sequence(
    // new InstantCommand(() -> updateLimits(maxTranslationalVelocity,
    // maxTranslationalAcceleration)),
    // driveToWaypoint(position));
    // }

    // private Command driveToWaypoint(Position position, AngularVelocity
    // maxRotationalVelocity) {
    // return Commands.sequence(
    // new InstantCommand(() -> updateLimits(maxRotationalVelocity)),
    // driveToWaypoint(position));
    // }

    // private Command driveToWaypoint(Position position, AngularVelocity
    // maxRotationalVelocity,
    // AngularVelocity maxRotationalAcceleration) {
    // return Commands.sequence(
    // new InstantCommand(
    // () -> updateLimits(maxRotationalVelocity, maxRotationalAcceleration)),
    // driveToWaypoint(position));
    // }

    // private Command driveToWaypoint(Position position, double
    // maxTranslationalVelocity,
    // AngularVelocity maxRotationalVelocity) {
    // return Commands.sequence(
    // new InstantCommand(() -> updateLimits(maxTranslationalVelocity,
    // maxRotationalVelocity)),
    // driveToWaypoint(position));
    // }

    private Command resetOdometry(Pose2d waypoint) {
        return Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(waypoint));
    }

    private Command resetOdometry(Position position) {
        return resetOdometry(AutoConstants.positionToPose.get(position));
    }

    private void updateLimits(double maxTranslationalVelocity, double maxTranslationalAcceleration,
            double maxRotationalVelocity, double maxRotationalAcceleration) {
        m_driveAngularVelocity.driveToPose(m_swerveSubsystem::getDriveToWaypoint,
                new ProfiledPIDController(
                        SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kP,
                        SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kI,
                        SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kD,
                        new TrapezoidProfile.Constraints(
                                maxTranslationalVelocity,
                                maxTranslationalAcceleration)),
                new ProfiledPIDController(
                        SwerveConstants.DRIVE_TO_POSE_ROTATION_kP,
                        SwerveConstants.DRIVE_TO_POSE_ROTATION_kI,
                        SwerveConstants.DRIVE_TO_POSE_ROTATION_kD,
                        new TrapezoidProfile.Constraints(
                                maxRotationalVelocity,
                                maxRotationalAcceleration)));
    }

    private void updateLimits(double maxTranslationalVelocity, double maxTranslationalAcceleration) {
        updateLimits(maxTranslationalVelocity, maxTranslationalAcceleration,
                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_VELOCITY_RAD,
                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD);
    }

    private void updateLimits(double maxTranslationalVelocity) {
        updateLimits(maxTranslationalVelocity, SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION);
    }

    private void updateLimits(AngularVelocity maxRotationalVelocity, AngularVelocity maxRotationalAcceleration) {
        updateLimits(SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_VELOCITY,
                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION,
                maxRotationalVelocity.in(RadiansPerSecond),
                maxRotationalAcceleration.in(RadiansPerSecond));
    }

    private void updateLimits(AngularVelocity maxRotationalVelocity) {
        updateLimits(maxRotationalVelocity,
                RadiansPerSecond.of(SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD));
    }

    private void updateLimits(double maxTranslationalVelocity, AngularVelocity maxRotationalVelocity) {
        updateLimits(maxTranslationalVelocity, SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION,
                maxRotationalVelocity.in(RadiansPerSecond),
                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD);
    }

//     public Command rightNeutralAuto() {

//         // TODO: Add alliance flipping util

//         SmartDashboard.putString("autoTest", "NOT STARTED");
//         SmartDashboard.putString("driveToWaypointCHECK", "NULL");

//         return new SequentialCommandGroup(
//                 resetOdometry(Position.STARTING_LINE_RIGHT),
//                 new InstantCommand(
//                         () -> m_robotContainer.driveAngularVelocity.driveToPoseEnabled(true)),

//                 new InstantCommand(() -> SmartDashboard.putString("autoTest", "GO TO NEUTRAL RIGHT 1")),

//                 // Drive to center line
//                 driveToWaypoint(Position.NEUTRAL_RIGHT_1),

//                 // Commands.parallel(
//                 // m_linearIntakeSubsystem.extend(),
//                 // m_intakeRollerSubsystem.intake(),
//                 // m_indexerSubsystem.run()),

//                 // Drive to center of field
//                 // driveToWaypoint(Position.NEUTRAL_RIGHT_2),

//                 // Drive to center field and extend/run intake

//                 // TODO: RE-ADD SHUFFLING (cause intake kicker bar is broken right now)

//                 new InstantCommand(() -> SmartDashboard.putString("autoTest", "GO TO NEUTRAL RIGHT 2")),

//                 driveToWaypoint(Position.NEUTRAL_RIGHT_2)
//                         .withTimeout(3)
//                         .deadlineFor(
//                                 m_linearIntakeSubsystem.extend(),
//                                 m_intakeRollerSubsystem.intake(),
//                                 m_indexerSubsystem.run()),

//                 new InstantCommand(() -> SmartDashboard.putString("autoTest", "GO TO NEUTRAL RIGHT 3")),

//                 Commands.parallel(
//                         driveToWaypoint(Position.NEUTRAL_RIGHT_3, Degrees.of(2.5)),
//                         m_linearIntakeSubsystem.midpoint().andThen(
//                                 m_intakeRollerSubsystem.stop(),
//                                 m_indexerSubsystem.stop()))
//                         .withTimeout(3),

//                 // .deadlineFor(
//                 // m_linearIntakeSubsystem.midpoint().andThen(
//                 // Commands.parallel(
//                 // m_intakeRollerSubsystem.stop(),
//                 // m_indexerSubsystem.stop()))),

//                 // Drive back to trench
//                 // Commands.deadline(
//                 // driveToWaypoint(Position.NEUTRAL_RIGHT_3),
//                 // m_linearIntakeSubsystem.midpoint().andThen(
//                 // Commands.parallel(
//                 // m_intakeRollerSubsystem.stop(),
//                 // m_indexerSubsystem.stop()))),

//                 new InstantCommand(
//                         () -> SmartDashboard.putString("autoTest", "GO TO ALLIANCE RIGHT 1")),

//                 // Get in shooting position
//                 driveToWaypoint(Position.ALLIANCE_RIGHT_1),

//                 new InstantCommand(() -> SmartDashboard.putString("autoTest", "SHOOT")),

//                 Commands.parallel(
//                         m_shooterSubsystem.aimAndShootIgnoreCheck(
//                                 m_swerveSubsystem::getDistanceToTarget),
//                         m_indexerSubsystem.run(),
//                         m_intakeRollerSubsystem.intake()));

//     }

    // Right neutral auto
    public AutoRoutine rightNeutralAuto() {
        AutoRoutine routine = m_autoFactory.newRoutine("rightNeutralAuto");

        AutoTrajectory trenchRightToNeutral = routine.trajectory("TrenchRightToNeutral");
        AutoTrajectory trenchRightToAlliance = routine.trajectory("TrenchRightToAlliance");

        routine.active().onTrue(
                Commands.sequence(
                        trenchRightToNeutral.resetOdometry(),
                        trenchRightToNeutral.cmd()
                )
        );

        trenchRightToNeutral.chain(trenchRightToAlliance);

        return routine;
    }

}
