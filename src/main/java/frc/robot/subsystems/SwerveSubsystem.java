// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    private Rotation2d autoAimTargetRotation = new Rotation2d();

    private Pose2d m_selectedClimbPose;

    private final PIDController m_choreoControllerX = new PIDController(10.0, 0.0, 0.0);
    private final PIDController m_choreoControllerY = new PIDController(10.0, 0.0, 0.0);
    private final PIDController m_choreoControllerHeading = new PIDController(7.5, 0.0, 0.0);

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(180));
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED, startingPose);
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
                                                 // via angle.
        swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
                                                // for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true,
                true,
                0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
                      // coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false,
                1); // Enable if you want to resynchronize your absolute encoders and motor encoders
                    // periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used
        // over the internal encoder and push the offsets onto it. Throws warning if not
        // possible

        if (isRedAlliance()) {
            m_selectedClimbPose = SwerveConstants.RED_LEFT_TOWER_CLIMB_POS;
        } else {
            m_selectedClimbPose = SwerveConstants.BLUE_LEFT_TOWER_CLIMB_POS;
        }

        m_choreoControllerHeading.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg,
                controllerCfg,
                SwerveConstants.MAX_SPEED,
                new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                        Rotation2d.fromDegrees(0)));
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Returns a Command that tells the robot to drive forward until the command
     * ends.
     *
     * @return a Command that tells the robot to drive forward until the command
     *         ends
     */
    public Command driveForward() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
        }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
    }

    /**
     * Returns a Command that tells the robot to drive backward until the command
     * ends.
     * 
     * @return a Command that tells the robot to drive backward until the command
     *         ends
     */
    public Command driveBackward() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(-1, 0), 0, false, false);
        }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
    }

    /**
     * Returns a Command that tells the robot to drive left until the command ends.
     * 
     * @return a Command that tells the robot to drive left until the command ends
     */
    public Command driveLeft() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(0, 1), 0, false, false);
        }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
    }

    /**
     * Returns a Command that tells the robot to drive right until the command ends.
     * 
     * @return a Command that tells the robot to drive right until the command ends
     */
    public Command driveRight() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(0, -1), 0, false, false);
        }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
     * object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     *                         controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     *                         controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     *                         smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                SwerveConstants.MAX_SPEED);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                SwerveConstants.MAX_SPEED);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Follows the given swerve sample by calculating the necessary chassis speeds
     * and
     * commanding them to the drive.
     * 
     * @param sample The swerve sample containing the desired velocities and
     *               heading.
     */
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + m_choreoControllerX.calculate(pose.getX(), sample.x),
                sample.vy + m_choreoControllerY.calculate(pose.getY(), sample.y),
                sample.omega + m_choreoControllerHeading.calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        driveFieldOriented(speeds);
    }

    // Left and right are from the perspective of the red/light driver station
    public enum Zone {
        BLUE_ALLIANCE_LEFT,
        BLUE_ALLIANCE_RIGHT,
        RED_ALLIANCE_LEFT,
        RED_ALLIANCE_RIGHT,
        NEUTRAL_ZONE_RED_LEFT,
        NEUTRAL_ZONE_RED_RIGHT,
        NEUTRAL_ZONE_BLUE_LEFT,
        NEUTRAL_ZONE_BLUE_RIGHT
    }

    /**
     * Gets the current zone of the robot based on its position on the field.
     * 
     * @return The current zone of the robot as a Zone enum.
     */
    public Zone getCurrentZone() {
        Translation2d position = getPose().getTranslation();

        if (position.getX() < FieldConstants.BLUE_STARTING_LINE_X) {
            if (position.getY() < FieldConstants.FIELD_WIDTH / 2) {
                return Zone.BLUE_ALLIANCE_RIGHT;
            } else {
                return Zone.BLUE_ALLIANCE_LEFT;
            }
        } else if (position.getX() > FieldConstants.RED_STARTING_LINE_X) {
            if (position.getY() < FieldConstants.FIELD_WIDTH / 2) {
                return Zone.RED_ALLIANCE_LEFT;
            } else {
                return Zone.RED_ALLIANCE_RIGHT;
            }
        } else if (position.getX() < (FieldConstants.FIELD_LENGTH / 2)) {
            if (position.getY() < FieldConstants.FIELD_WIDTH / 2) {
                return Zone.NEUTRAL_ZONE_BLUE_RIGHT;
            } else {
                return Zone.NEUTRAL_ZONE_BLUE_LEFT;
            }
        } else {
            if (position.getY() < FieldConstants.FIELD_WIDTH / 2) {
                return Zone.NEUTRAL_ZONE_RED_LEFT;
            } else {
                return Zone.NEUTRAL_ZONE_RED_RIGHT;
            }
        }
    }

    /**
     * Gets the distance to the hub.
     *
     * @return the distance to the hub to the center of the robot in meters
     */
    public Distance getDistanceToTarget() {
        Translation2d position = getPose().getTranslation();
        return Meters.of(position.getDistance(getAutoAimTarget()));
    }

    /**
     * Gets the target translation for auto-aiming based on the current zone and
     * alliance.
     * 
     * @return The target translation for auto-aiming.
     */
    private Translation2d getAutoAimTarget() {
        if (isRedAlliance()) {
            switch (getCurrentZone()) {
                case NEUTRAL_ZONE_RED_LEFT:
                case NEUTRAL_ZONE_BLUE_RIGHT:
                case BLUE_ALLIANCE_RIGHT:
                    return SwerveConstants.RED_LEFT_FEEDING_TARGET;
                case NEUTRAL_ZONE_RED_RIGHT:
                case NEUTRAL_ZONE_BLUE_LEFT:
                case BLUE_ALLIANCE_LEFT:
                    return SwerveConstants.RED_RIGHT_FEEDING_TARGET;
                case RED_ALLIANCE_LEFT:
                case RED_ALLIANCE_RIGHT:
                    return FieldConstants.RED_HUB_CENTER;
            }
        } else {
            switch (getCurrentZone()) {
                case NEUTRAL_ZONE_RED_LEFT:
                case NEUTRAL_ZONE_BLUE_RIGHT:
                case RED_ALLIANCE_RIGHT:
                    return SwerveConstants.BLUE_LEFT_FEEDING_TARGET;
                case NEUTRAL_ZONE_RED_RIGHT:
                case NEUTRAL_ZONE_BLUE_LEFT:
                case RED_ALLIANCE_LEFT:
                    return SwerveConstants.BLUE_RIGHT_FEEDING_TARGET;
                case BLUE_ALLIANCE_LEFT:
                case BLUE_ALLIANCE_RIGHT:
                    return FieldConstants.BLUE_HUB_CENTER;
            }
        }

        // Default to hub center if something goes wrong
        return isRedAlliance()
                ? FieldConstants.RED_HUB_CENTER
                : FieldConstants.BLUE_HUB_CENTER;
    }

    /**
     * Calculates the target rotation for auto-aiming based on the current position
     * and
     * velocity of the robot, and the target translation for auto-aiming.
     */
    private void calculateAutoAimHeading() {
        Translation2d robotTranslation = getPose().getTranslation();

        ChassisSpeeds fieldRelativeChassisSpeeds = getFieldVelocity();

        Translation2d deltaPos = new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond,
                fieldRelativeChassisSpeeds.vyMetersPerSecond);

        Translation2d futureTranslation = robotTranslation
                .plus(deltaPos.times(SwerveConstants.AUTO_AIM_VELOCITY_COMPENSATION_FACTOR));

        Translation2d target = getAutoAimTarget();

        Translation2d delta = target.minus(futureTranslation);
        autoAimTargetRotation = delta.getAngle().plus(Rotation2d.fromDegrees(90));
    }

    /**
     * Checks if the robot is on target for auto-aiming by comparing the current
     * heading
     * to the target rotation for auto-aiming, and checking if the angle error is
     * within a certain tolerance.
     * 
     * @return true if the robot is on target for auto-aiming, false otherwise
     */
    public boolean isAutoAimOnTarget() {
        Rotation2d currentHeading = getHeading();
        double angleError = Math
                .abs(currentHeading.minus(autoAimTargetRotation.plus(Rotation2d.fromDegrees(90))).getDegrees());
        SmartDashboard.putNumber("angleError", angleError);
        return angleError < SwerveConstants.AUTO_AIM_ANGLE_TOLERANCE.in(Degrees);
    }

    /**
     * Gets the target rotation for auto-aiming.
     * 
     * @return The target rotation for auto-aiming.
     */
    public Rotation2d getAutoAimHeading() {
        calculateAutoAimHeading();
        return autoAimTargetRotation;
    }

    public void setSelectedClimbPose(boolean isLeft) {
        if (isRedAlliance()) {
            m_selectedClimbPose = isLeft ? SwerveConstants.RED_LEFT_TOWER_CLIMB_POS
                    : SwerveConstants.RED_RIGHT_TOWER_CLIMB_POS;
        } else {
            m_selectedClimbPose = isLeft ? SwerveConstants.BLUE_LEFT_TOWER_CLIMB_POS
                    : SwerveConstants.BLUE_RIGHT_TOWER_CLIMB_POS;
        }
    }

    public Pose2d getSelectedClimbPose() {
        return m_selectedClimbPose;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putBoolean("isAutoAimReady", isAutoAimOnTarget());
    }
}