// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
    private final TalonFX m_linearMotor;
    private final SmartMotorController m_linearMotorSMC;
    private final SmartMotorControllerConfig m_linearMotorSMCConfig;
    private final Elevator m_linearIntake;

    public LinearIntakeSubsystem() {
        m_linearMotor = new TalonFX(LinearIntakeConstants.MOTOR_CAN_ID);

        m_linearMotorSMCConfig = new SmartMotorControllerConfig(this)
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
                .withMotorInverted(false)
                .withClosedLoopRampRate(LinearIntakeConstants.CLOSED_LOOP_RAMP_RATE)
                .withOpenLoopRampRate(LinearIntakeConstants.OPEN_LOOP_RAMP_RATE)
                .withFeedforward(LinearIntakeConstants.FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_linearMotorSMC = new TalonFXWrapper(
                m_linearMotor,
                LinearIntakeConstants.MOTOR,
                m_linearMotorSMCConfig);

        MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(MechanismPositionConstants.ROBOT_MAX_HEIGHT)
                .withMaxRobotLength(MechanismPositionConstants.ROBOT_MAX_LENGTH)
                .withRelativePosition(LinearIntakeConstants.RELATIVE_POSITION);

        ElevatorConfig m_linearConfig = new ElevatorConfig(m_linearMotorSMC)
                .withStartingHeight(LinearIntakeConstants.STARTING_HEIGHT)
                .withHardLimits(
                        LinearIntakeConstants.HARD_LIMIT_MIN,
                        LinearIntakeConstants.HARD_LIMIT_MAX)
                .withTelemetry("LinearIntake", Constants.TELEMETRY_VERBOSITY)
                .withMechanismPositionConfig(m_robotToMechanism)
                .withMass(LinearIntakeConstants.MECHANISM_MASS)
                .withAngle(LinearIntakeConstants.MECHANISM_ANGLE);

        m_linearIntake = new Elevator(m_linearConfig);
    }

    public Command sysId() {
        return m_linearIntake.sysId(
                Volts.of(12), Volts.of(12).per(Second), Second.of(30))
                .beforeStarting(SignalLogger::start)
                .finallyDo(SignalLogger::stop);
    }

    public Command setLinearPosition(Distance position) {
        return m_linearIntake.setHeight(position);
    }

    public boolean isLinearAtTargetPosition() {
        Optional<Angle> angle_setpoint = m_linearIntake.getMechanismSetpoint();
        if (!angle_setpoint.isPresent()) {
            return false;
        }
        Distance setpoint = m_linearMotorSMCConfig.convertFromMechanism(angle_setpoint.get());
        return setpoint.isNear(m_linearIntake.getHeight(), LinearIntakeConstants.POSITION_TARGET_ERROR);
    }

    public Command elevCmd(double dutycycle) {
        return m_linearIntake.set(dutycycle);
    }

    public Command extend() {
        return setLinearPosition(LinearIntakeConstants.EXTENDED_POSITION).until(this::isLinearAtTargetPosition);
    }

    public Command retract() {
        return setLinearPosition(LinearIntakeConstants.RETRACTED_POSITION).until(this::isLinearAtTargetPosition);
    }

    public Command fullyRetract() {
        return setLinearPosition(LinearIntakeConstants.FULLY_RETRACTED_POSITION).until(this::isLinearAtTargetPosition);
    }

    public Command set(double dutycycle) {
        return m_linearIntake.set(dutycycle);
    }

    public enum LinearIntakePosition {
        EXTENDED, RETRACTED, FULLY_RETRACTED
    }

    public LinearIntakePosition getCurrentPosition() {
        Distance currentPosition = m_linearIntake.getHeight();

        if (currentPosition.isNear(LinearIntakeConstants.EXTENDED_POSITION,
                LinearIntakeConstants.POSITION_TARGET_ERROR)) {
            return LinearIntakePosition.EXTENDED;
        } else if (currentPosition.isNear(LinearIntakeConstants.RETRACTED_POSITION,
                LinearIntakeConstants.POSITION_TARGET_ERROR)) {
            return LinearIntakePosition.RETRACTED;
        } else if (currentPosition.isNear(LinearIntakeConstants.FULLY_RETRACTED_POSITION,
                LinearIntakeConstants.POSITION_TARGET_ERROR)) {
            return LinearIntakePosition.FULLY_RETRACTED;
        } else {
            return null; // or throw an exception, or return an Optional
        }
    }

    @Override
    public void periodic() {
        m_linearIntake.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_linearIntake.simIterate();
    }
}
