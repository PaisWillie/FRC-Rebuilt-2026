// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

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
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

public class FuelSimSubsystem extends SubsystemBase {
  /** Creates a new FuelSimSubsystem. */
  public FuelSimSubsystem() {
  }

  public static Command shootFuel(Supplier<LinearVelocity> flywheelLinearVelocity, Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeedsFieldRelative, Supplier<Rotation2d> heading, Supplier<Angle> hoodAngle) {

    return Commands.runOnce(
        () -> {

          if (!RobotBase.isSimulation())
            return;

          RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
              // Specify the position of the chassis when the note is launched
              robotPose.get().getTranslation(),
              // Specify the translation of the shooter from the robot center (in the
              // shooter’s reference frame)
              new Translation2d(Inches.of(-5.087),
                  Inches.of(0)),
              // Specify the field-relative speed of the chassis, adding it to the initial
              // velocity of the projectile
              chassisSpeedsFieldRelative.get(),
              // The shooter facing direction is the same as the robot’s facing direction
              heading.get(),
              // Initial height of the flying fuel
              Inches.of(21.75),
              // The launch speed is proportional to the RPM; assumed to be 20 meters/second
              // at 6000 RPM
              flywheelLinearVelocity.get(),
              // MetersPerSecond.of(5),
              // The angle at which the fuel is launched
              Degrees.of(90).plus(hoodAngle.get()));

          fuelOnFly
              // Set the target center to the Rebbuilt Hub of the current alliance
              .withTargetPosition(
                  () -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
              // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
              // speaker's "mouth")
              .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3));

          SimulatedArena.getInstance().addGamePieceProjectile(
              fuelOnFly);
        });
  }

  StructArrayPublisher<Pose3d> fuelPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct)
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
