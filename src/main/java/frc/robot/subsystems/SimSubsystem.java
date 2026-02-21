// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

public class SimSubsystem extends SubsystemBase {
    /** Creates a new FuelSimSubsystem. */
    private final IntakeSimulation intakeSimulation;

    public SimSubsystem(AbstractDriveTrainSimulation driveTrainSimulation) {
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Fuel",
                // Specify the drivetrain to which this intake is attached
                driveTrainSimulation,
                // Width of the intake
                Meters.of(0.7),
                // The extension length of the intake beyond the robot's frame (when activated)
                Meters.of(0.2),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.FRONT,
                // The intake can hold up to 25 fuel
                25);

        FieldConstants.FUEL_LOCATIONS.forEach((fuel) -> SimulatedArena.getInstance().addGamePiece(
                new RebuiltFuelOnField(fuel)));
    }

    public Command startIntake() {
        return Commands.runOnce(() -> intakeSimulation.startIntake());
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> intakeSimulation.stopIntake());
    }

    public Command shootFuel(Supplier<LinearVelocity> flywheelLinearVelocity, Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> chassisSpeedsFieldRelative, Supplier<Rotation2d> heading,
            Supplier<Angle> hoodAngle, Supplier<Boolean> isRedAlliance) {

        return Commands.runOnce(
                () -> {

                    if (!RobotBase.isSimulation())
                        return;

                    if (!intakeSimulation.obtainGamePieceFromIntake())
                        return;

                    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                            // Specify the position of the chassis when the note is launched
                            robotPose.get().getTranslation(),
                            // Specify the translation of the shooter from the robot center
                            // (in the
                            // shooter’s reference frame)
                            new Translation2d(Inches.of(-5.087),
                                    Inches.of(0)),
                            // Specify the field-relative speed of the chassis, adding it to
                            // the initial
                            // velocity of the projectile
                            chassisSpeedsFieldRelative.get(),
                            // The shooter facing direction is the same as the robot’s
                            // facing direction
                            heading.get(),
                            // Initial height of the flying fuel
                            Inches.of(21.75),
                            // The launch speed is proportional to the RPM; assumed to be 20
                            // meters/second
                            // at 6000 RPM
                            flywheelLinearVelocity.get(),
                            // MetersPerSecond.of(5),
                            // The angle at which the fuel is launched
                            Degrees.of(90).plus(hoodAngle.get()));

                    fuelOnFly
                            // Set the target center to the ReBuilt Hub of the current alliance
                            .withTargetPosition(
                                    () -> {
                                        Translation2d target = isRedAlliance.get() ? FieldConstants.RED_HUB_CENTER
                                                : FieldConstants.BLUE_HUB_CENTER;
                                        return new Translation3d(target.getX(), target.getY(),
                                                FieldConstants.HUB_HEIGHT);
                                    })
                            // Set the taraget tolerance
                            .withTargetTolerance(
                                    new Translation3d(FieldConstants.HUB_WIDTH, FieldConstants.HUB_WIDTH, 0.5))
                            .withHitTargetCallBack(() -> {
                                outputFuelFromHub(isRedAlliance);
                                System.out.println("Hit target!");
                            });

                    SimulatedArena.getInstance().addGamePieceProjectile(
                            fuelOnFly);
                });
    }

    private void outputFuelFromHub(Supplier<Boolean> isRedAlliance) {

        Translation2d hubLocation = isRedAlliance.get() ? FieldConstants.RED_HUB_CENTER
                : FieldConstants.BLUE_HUB_CENTER;

        Translation2d hubOutput = hubLocation.minus(new Translation2d(FieldConstants.HUB_WIDTH / 2, 0));

        Rotation2d hubOutputRotation = isRedAlliance.get() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

        SimulatedArena.getInstance().addGamePieceProjectile(
                new RebuiltFuelOnFly(
                        hubOutput,
                        new Translation2d(0, 0),
                        new ChassisSpeeds(0, 0, 0),
                        hubOutputRotation,
                        Meters.of(FieldConstants.HUB_HEIGHT / 2),
                        MetersPerSecond.of(3),
                        Degrees.of(-30)));
    }

    StructArrayPublisher<Pose3d> fuelPoses = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Fuel", Pose3d.struct)
            .publish();

    @Override
    public void periodic() {
        // Get the positions of the fuel (both on the field and in the air)
        // fuelPoses.accept(SimulatedArena.getInstance()
        // .getGamePiecesArrayByType("Fuel")
        // .toArray(Pose3d[]::new));

        // Pose3d[] fuelPoses = SimulatedArena.getInstance()
        // .getGamePiecesArrayByType("Fuel");
        this.fuelPoses.accept(SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel"));
    }
}
