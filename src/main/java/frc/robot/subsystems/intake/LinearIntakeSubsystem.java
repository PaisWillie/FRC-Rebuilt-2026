// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.LinearIntakeConstants;
import frc.robot.Constants.MechanismPositionConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class LinearIntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final SmartMotorController m_smartMotorController;
    private final SmartMotorControllerConfig m_smcConfig;
    private final Elevator m_linearIntake;

    private final DigitalInput m_extendedLimitSwitch;
    private final DigitalInput m_retractedLimitSwitch;

    public LinearIntakeSubsystem() {
        m_motor = new TalonFX(LinearIntakeConstants.MOTOR_CAN_ID);

        m_smcConfig = new SmartMotorControllerConfig(this)
                .withMechanismCircumference(LinearIntakeConstants.MOTOR_CIRCUMFERENCE)
                .withClosedLoopController(
                        LinearIntakeConstants.PID_kP,
                        LinearIntakeConstants.PID_kI,
                        LinearIntakeConstants.PID_kD,
                        LinearIntakeConstants.MAX_VELOCITY,
                        LinearIntakeConstants.MAX_ACCELERATION)
                .withSimClosedLoopController(
                        LinearIntakeConstants.SIM_PID_kP,
                        LinearIntakeConstants.SIM_PID_kI,
                        LinearIntakeConstants.SIM_PID_kD,
                        LinearIntakeConstants.MAX_VELOCITY,
                        LinearIntakeConstants.MAX_ACCELERATION)
                .withSoftLimit(
                        LinearIntakeConstants.SOFT_LIMIT_MIN,
                        LinearIntakeConstants.SOFT_LIMIT_MAX)
                .withGearing(new MechanismGearing(LinearIntakeConstants.GEARBOX))
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("LinearIntakeMotor", Constants.TELEMETRY_VERBOSITY)
                .withStatorCurrentLimit(LinearIntakeConstants.STATOR_CURRENT_LIMIT)
                .withMotorInverted(LinearIntakeConstants.INVERT_MOTOR)
                .withClosedLoopRampRate(LinearIntakeConstants.CLOSED_LOOP_RAMP_RATE)
                .withOpenLoopRampRate(LinearIntakeConstants.OPEN_LOOP_RAMP_RATE)
                .withFeedforward(LinearIntakeConstants.FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_smartMotorController = new TalonFXWrapper(
                m_motor,
                LinearIntakeConstants.MOTOR,
                m_smcConfig);

        MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(MechanismPositionConstants.ROBOT_MAX_HEIGHT)
                .withMaxRobotLength(MechanismPositionConstants.ROBOT_MAX_LENGTH)
                .withRelativePosition(LinearIntakeConstants.RELATIVE_POSITION);

        ElevatorConfig m_linearConfig = new ElevatorConfig(m_smartMotorController)
                .withStartingHeight(LinearIntakeConstants.RETRACTED_POSITION)
                .withHardLimits(
                        LinearIntakeConstants.HARD_LIMIT_MIN,
                        LinearIntakeConstants.HARD_LIMIT_MAX)
                .withTelemetry("LinearIntakeMech", Constants.TELEMETRY_VERBOSITY)
                .withMechanismPositionConfig(m_robotToMechanism)
                .withMass(LinearIntakeConstants.MECHANISM_MASS)
                .withHorizontalElevator();

        m_linearIntake = new Elevator(m_linearConfig);

        m_extendedLimitSwitch = new DigitalInput(LinearIntakeConstants.EXTENDED_LIMIT_SWITCH_DIO);
        m_retractedLimitSwitch = new DigitalInput(LinearIntakeConstants.RETRACTED_LIMIT_SWITCH_DIO);
    }

    public Command sysId() {
        return m_linearIntake.sysId(
                Volts.of(3), Volts.of(3).per(Second), Second.of(10))
                .beforeStarting(SignalLogger::start)
                .finallyDo(SignalLogger::stop);
    }

    public Command setPosition(Distance position) {
        return m_linearIntake.setHeight(position);
    }

    public Distance getPosition() {
        return m_linearIntake.getHeight();
    }

    public Optional<Distance> getSetpoint() {
        Optional<Angle> angle_setpoint = m_linearIntake.getMechanismSetpoint();
        if (!angle_setpoint.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(m_smcConfig.convertFromMechanism(angle_setpoint.get()));
    }

    public boolean isAtTargetPosition() {
        Optional<Angle> angle_setpoint = m_linearIntake.getMechanismSetpoint();

        if (!angle_setpoint.isPresent()) {
            return false;
        }

        return getSetpoint().map(
                setpoint -> setpoint.isNear(m_linearIntake.getHeight(), LinearIntakeConstants.POSITION_TARGET_ERROR))
                .orElse(false);
    }

    public Command elevCmd(double dutycycle) {
        return m_linearIntake.set(dutycycle);
    }

    public Command extend() {
        return setPosition(LinearIntakeConstants.EXTENDED_POSITION).until(this::isAtTargetPosition);
    }

    public Command retract() {
        return setPosition(LinearIntakeConstants.MIDPOINT_POSITION).until(this::isAtTargetPosition);
    }

    public Command fullyRetract() {
        return setPosition(LinearIntakeConstants.RETRACTED_POSITION).until(this::isAtTargetPosition);
    }

    public Command set(double dutycycle) {
        return m_linearIntake.set(dutycycle);
    }

    public enum LinearIntakePosition {
        EXTENDED, RETRACTED, FULLY_RETRACTED, UNKNOWN
    }

    public LinearIntakePosition getCurrentPositionEnum() {
        Distance currentPosition = m_linearIntake.getHeight();

        if (currentPosition.isNear(LinearIntakeConstants.EXTENDED_POSITION,
                LinearIntakeConstants.POSITION_TARGET_ERROR)) {
            return LinearIntakePosition.EXTENDED;
        } else if (currentPosition.isNear(LinearIntakeConstants.MIDPOINT_POSITION,
                LinearIntakeConstants.POSITION_TARGET_ERROR)) {
            return LinearIntakePosition.RETRACTED;
        } else if (currentPosition.isNear(LinearIntakeConstants.RETRACTED_POSITION,
                LinearIntakeConstants.POSITION_TARGET_ERROR)) {
            return LinearIntakePosition.FULLY_RETRACTED;
        } else {
            return LinearIntakePosition.UNKNOWN;
        }
    }

    public boolean getExtendedLimitSwitch() {
        return m_extendedLimitSwitch.get();
    }

    public boolean getRetractedLimitSwitch() {
        return m_retractedLimitSwitch.get();
    }

    private void handleLimitSwitches() {
        if (getExtendedLimitSwitch()) {
            m_smartMotorController.setEncoderPosition(LinearIntakeConstants.EXTENDED_POSITION);
        } else if (getRetractedLimitSwitch()) {
            m_smartMotorController.setEncoderPosition(LinearIntakeConstants.MIDPOINT_POSITION);
        }
    }

    @Override
    public void periodic() {
        m_linearIntake.updateTelemetry();

        handleLimitSwitches();

        if (Constants.TELEMETRY) {
            SmartDashboard.putNumber("LinearIntakeMech/position (m)", getPosition().in(Meters));
            SmartDashboard.putNumber("LinearIntakeMech/setpoint (m)",
                    getSetpoint().map(pos -> pos.in(Meters)).orElse(Double.NaN));
            SmartDashboard.putString("LinearIntakeMech/currentPositionEnum", getCurrentPositionEnum().name());
        }
    }

    @Override
    public void simulationPeriodic() {
        m_linearIntake.simIterate();
    }

}