// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MechanismPositionConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    private final SmartMotorController m_smartMotorController;

    private final Arm m_hood;

    private final Debouncer m_atAngleDebouncer = new Debouncer(HoodConstants.AT_ANGLE_DEBOUNCE_TIME,
            Debouncer.DebounceType.kRising);

    private final InterpolatingDoubleTreeMap m_distanceToHoodAngleMap;

    public HoodSubsystem() {
        m_motor = new TalonFX(HoodConstants.MOTOR_CAN_ID);
        m_encoder = new CANcoder(HoodConstants.ENCODER_CAN_ID);

        SmartMotorControllerConfig m_smcConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(
                        HoodConstants.PID_kP,
                        HoodConstants.PID_kI,
                        HoodConstants.PID_kD,
                        HoodConstants.MAX_VELOCITY_RPM,
                        HoodConstants.MAX_ACCELERATION_RPS2)
                .withSimClosedLoopController(
                        HoodConstants.SIM_PID_kP,
                        HoodConstants.SIM_PID_kI,
                        HoodConstants.SIM_PID_kD,
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

        MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(MechanismPositionConstants.ROBOT_MAX_HEIGHT)
                .withMaxRobotLength(MechanismPositionConstants.ROBOT_MAX_LENGTH)
                .withRelativePosition(HoodConstants.RELATIVE_POSITION);

        // The Hood can be modeled as an arm since it has a gravitational force acting
        // on it
        ArmConfig m_hoodConfig = new ArmConfig(m_smartMotorController)
                .withTelemetry("HoodMech", Constants.TELEMETRY_VERBOSITY)
                .withMechanismPositionConfig(m_robotToMechanism)
                .withSoftLimits(HoodConstants.SOFT_LIMIT_MIN, HoodConstants.SOFT_LIMIT_MAX)
                .withHardLimit(HoodConstants.HARD_LIMIT_MIN, HoodConstants.HARD_LIMIT_MAX)
                .withLength(HoodConstants.LENGTH)
                .withMass(HoodConstants.MASS);

        m_hood = new Arm(m_hoodConfig);

        m_distanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
        HoodConstants.SHOOTER_DISTANCE_TO_HOOD_ANGLE.forEach(m_distanceToHoodAngleMap::put);
    }

    /**
     * Creates a SysId characterization command for the hood.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_hood.sysId(
                Volts.of(4), Volts.of(0.5).per(Second), Second.of(8))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
    }

    /**
     * Creates a command to set the hood angle.
     *
     * @param angle the target angle
     * @return the command that sets the angle
     */
    public Command setAngle(Angle angle) {
        return m_hood.setAngle(angle);
    }

    /**
     * Directly sets the hood angle.
     *
     * @param angle the target angle
     */
    public void setAngleDirect(Angle angle) {
        m_smartMotorController.setPosition(angle);
    }

    /**
     * Creates a command to set the hood angle from a supplier.
     *
     * @param angleSupplier the supplier of target angle
     * @return the command that sets the angle
     */
    public Command setAngle(Supplier<Angle> angleSupplier) {
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

    public boolean isAtTargetAngle() {
        // TODO: Maybe change this to get setpoint from SMC instead of hood
        Optional<Angle> setpoint = m_hood.getMechanismSetpoint();

        if (!setpoint.isPresent())
            return false;

        return m_atAngleDebouncer.calculate(
                setpoint.get().isNear(m_hood.getAngle(), HoodConstants.ANGLE_TARGET_ERROR));
    }

    /**
     * Gets the target hood angle based on the distance to the target using
     * interpolation.
     * 
     * @param distanceToTarget the distance from the robot to the target
     * @return the target hood angle
     */
    public Angle getAngleToTarget(Distance distanceToTarget) {
        return Degrees.of(m_distanceToHoodAngleMap.get(distanceToTarget.in(Meters)));
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

    /**
     * Lowers the hood to the default angle, to avoid hitting the hood on the trench
     * when not shooting.
     * 
     * @return the command that lowers the hood
     */
    public Command lowerHood() {
        return m_hood.setAngle(HoodConstants.DEFAULT_ANGLE);
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
