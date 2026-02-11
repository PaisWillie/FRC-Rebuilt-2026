// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Degrees;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(HoodConstants.MOTOR_CAN_ID);
    private final CANcoder m_encoder = new CANcoder(HoodConstants.ENCODER_CAN_ID);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                    HoodConstants.PID_kP,
                    HoodConstants.PID_kI,
                    HoodConstants.PID_kD,
                    HoodConstants.MAX_VELOCITY_RPM,
                    HoodConstants.MAX_ACCELERATION_RPS2)
            .withGearing(new MechanismGearing(HoodConstants.GEARBOX))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMotor", Constants.TELEMETRY_VERBOSITY)
            .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT_AMPS)
            .withMotorInverted(false)
            .withClosedLoopRampRate(HoodConstants.CLOSED_LOOP_RAMP_RATE_SEC)
            .withOpenLoopRampRate(HoodConstants.OPEN_LOOP_RAMP_RATE_SEC)
            .withFeedforward(HoodConstants.FEEDFORWARD)
            .withSimFeedforward(HoodConstants.SIM_FEEDFORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withExternalEncoder(m_encoder)
            .withExternalEncoderInverted(HoodConstants.EXTERNAL_ENCODER_INVERTED)
            .withExternalEncoderGearing(HoodConstants.EXTERNAL_ENCODER_GEARING)
            .withExternalEncoderZeroOffset(HoodConstants.EXTERNAL_ENCODER_ZERO_OFFSET)
            .withUseExternalFeedbackEncoder(true);

    private final SmartMotorController smartMotorController = new TalonFXWrapper(
            m_motor,
            HoodConstants.MOTOR,
            smcConfig);

    private final ArmConfig hoodConfig = new ArmConfig(smartMotorController)
            .withTelemetry("HoodMech", Constants.TELEMETRY_VERBOSITY)
            .withSoftLimits(HoodConstants.SOFT_LIMIT_MIN, HoodConstants.SOFT_LIMIT_MAX)
            .withHardLimit(HoodConstants.HARD_LIMIT_MIN, HoodConstants.HARD_LIMIT_MAX); // The Hood can be modeled as an
                                                                                        // arm since it has a

    // We use Arm, since a hood has gravitational force acted upon based on the
    // angle its in
    private final Arm hood = new Arm(hoodConfig);

    public HoodSubsystem() {

    }

    /**
     * Creates a command to set the hood angle.
     *
     * @param angle the target angle
     * @return the command that sets the angle
     */
    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    /**
     * Directly sets the hood angle.
     *
     * @param angle the target angle
     */
    public void setAngleDirect(Angle angle) {
        smartMotorController.setPosition(angle);
    }

    /**
     * Creates a command to set the hood angle from a supplier.
     *
     * @param angleSupplier the supplier of target angle
     * @return the command that sets the angle
     */
    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    /**
     * Gets the current hood angle.
     *
     * @return the current angle
     */
    public Angle getAngle() {
        return hood.getAngle();
    }

    /**
     * Creates a SysId characterization command for the hood.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return hood.sysId(
                HoodConstants.SYSID_MAX_VOLTAGE,
                HoodConstants.SYSID_STEP,
                HoodConstants.SYSID_DURATION);
    }

    /**
     * Moves the hood up and down.
     *
     * @param dutyCycleSupplier the supplier of [-1, 1] speed to set the hood to.
     * @return the command that sets duty cycle
     */
    public Command set(Supplier<Double> dutyCycleSupplier) {
        return hood.set(dutyCycleSupplier);
    }

    /**
     * Move the hood up and down.
     * 
     * @param dutyCycle [-1, 1] speed to set the hood to.
     * @return the command that sets the duty cycle
     */

    public Command set(double dutyCycle) {
        return hood.set(dutyCycle);
    }

    /**
     * Updates hood telemetry.
     */
    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    /**
     * Runs the hood simulation step.
     */
    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
