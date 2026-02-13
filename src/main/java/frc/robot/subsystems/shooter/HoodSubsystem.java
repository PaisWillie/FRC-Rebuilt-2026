// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    private final SmartMotorControllerConfig m_smcConfig;

    private final SmartMotorController m_smartMotorController;

    // The Hood can be modeled as an arm since it has a gravitational force acting
    // on it
    private final ArmConfig m_hoodConfig;

    private final Arm m_hood;

    private Angle m_targetAngle = HoodConstants.STARTING_ANGLE;

    public HoodSubsystem() {
        m_motor = new TalonFX(HoodConstants.MOTOR_CAN_ID);
        m_encoder = new CANcoder(HoodConstants.ENCODER_CAN_ID);

        m_smcConfig = new SmartMotorControllerConfig(this)
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

        m_smartMotorController = new TalonFXWrapper(
                m_motor,
                HoodConstants.MOTOR,
                m_smcConfig);

        m_hoodConfig = new ArmConfig(m_smartMotorController)
                .withTelemetry("HoodMech", Constants.TELEMETRY_VERBOSITY)
                .withSoftLimits(HoodConstants.SOFT_LIMIT_MIN, HoodConstants.SOFT_LIMIT_MAX)
                .withHardLimit(HoodConstants.HARD_LIMIT_MIN, HoodConstants.HARD_LIMIT_MAX)
                .withLength(HoodConstants.LENGTH)
                .withMass(HoodConstants.MASS)
                .withStartingPosition(HoodConstants.STARTING_ANGLE);

        m_hood = new Arm(m_hoodConfig);
    }

    /**
     * Creates a command to set the hood angle.
     *
     * @param angle the target angle
     * @return the command that sets the angle
     */
    public Command setAngle(Angle angle) {
        m_targetAngle = angle;
        return m_hood.setAngle(angle);
    }

    /**
     * Directly sets the hood angle.
     *
     * @param angle the target angle
     */
    public void setAngleDirect(Angle angle) {
        m_targetAngle = angle;
        m_smartMotorController.setPosition(angle);
    }

    /**
     * Creates a command to set the hood angle from a supplier.
     *
     * @param angleSupplier the supplier of target angle
     * @return the command that sets the angle
     */
    public Command setAngle(Supplier<Angle> angleSupplier) {
        m_targetAngle = angleSupplier.get();
        return m_hood.setAngle(angleSupplier);
    }

    /**
     * Gets the current hood angle.
     *
     * @return the current angle
     */
    public Angle getAngle() {
        return m_hood.getAngle();
    }

    public boolean isAtAngle() {
        double deltaDeg = m_targetAngle.minus(m_hood.getAngle()).in(Units.Degrees);
        return Math.abs(deltaDeg) <= HoodConstants.ANGLE_TOLERANCE.in(Units.Degrees);
    }

    /**
     * Creates a SysId characterization command for the hood.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_hood.sysId(
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
        return m_hood.set(dutyCycleSupplier);
    }

    /**
     * Move the hood up and down.
     * 
     * @param dutyCycle [-1, 1] speed to set the hood to.
     * @return the command that sets the duty cycle
     */

    public Command set(double dutyCycle) {
        return m_hood.set(dutyCycle);
    }

    public Command lowerHood() {
        return m_hood.setAngle(HoodConstants.SOFT_LIMIT_MIN);
    }

    /**
     * Updates hood telemetry.
     */
    @Override
    public void periodic() {
        m_hood.updateTelemetry();
    }

    /**
     * Runs the hood simulation step.
     */
    @Override
    public void simulationPeriodic() {
        m_hood.simIterate();
    }
}
