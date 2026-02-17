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
        .withMechanismCircumference(LinearIntakeConstants.MOTOR_CIRCUMFERENCE_METERS)
        .withClosedLoopController(
            LinearIntakeConstants.PID_kP,
            LinearIntakeConstants.PID_kI,
            LinearIntakeConstants.PID_kD,
            LinearIntakeConstants.MAX_VELOCITY_MPS,
            LinearIntakeConstants.MAX_ACCELERATION_MPS2)
        .withSimClosedLoopController(
            LinearIntakeConstants.SIM_PID_kP,
            LinearIntakeConstants.SIM_PID_kI,
            LinearIntakeConstants.SIM_PID_kD,
            LinearIntakeConstants.MAX_VELOCITY_MPS,
            LinearIntakeConstants.MAX_ACCELERATION_MPS2)
        .withSoftLimit(
            LinearIntakeConstants.SOFT_LIMIT_MIN_METERS,
            LinearIntakeConstants.SOFT_LIMIT_MAX_METERS)
        .withGearing(new MechanismGearing(LinearIntakeConstants.GEARBOX))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("LinearIntakeMotor", Constants.TELEMETRY_VERBOSITY)
        .withStatorCurrentLimit(LinearIntakeConstants.STATOR_CURRENT_LIMIT_AMPS)
        .withMotorInverted(false)
        .withClosedLoopRampRate(LinearIntakeConstants.CLOSED_LOOP_RAMP_RATE_SEC)
        .withOpenLoopRampRate(LinearIntakeConstants.OPEN_LOOP_RAMP_RATE_SEC)
        .withFeedforward(LinearIntakeConstants.FEEDFORWARD)
        .withControlMode(ControlMode.CLOSED_LOOP);

    m_linearMotorSMC = new TalonFXWrapper(
        m_linearMotor,
        LinearIntakeConstants.MOTOR,
        m_linearMotorSMCConfig);

    MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
        .withMaxRobotHeight(LinearIntakeConstants.ROBOT_MAX_HEIGHT_METERS)
        .withMaxRobotLength(LinearIntakeConstants.ROBOT_MAX_LENGTH_METERS)
        .withRelativePosition(LinearIntakeConstants.RELATIVE_POSITION_METERS);

    ElevatorConfig m_linearConfig = new ElevatorConfig(m_linearMotorSMC)
        .withStartingHeight(LinearIntakeConstants.STARTING_HEIGHT_METERS)
        .withHardLimits(
            LinearIntakeConstants.HARD_LIMIT_MIN_METERS,
            LinearIntakeConstants.HARD_LIMIT_MAX_METERS)
        .withTelemetry("LinearIntake", Constants.TELEMETRY_VERBOSITY)
        .withMechanismPositionConfig(m_robotToMechanism)
        .withMass(LinearIntakeConstants.MECHANISM_MASS_POUNDS);

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
    return setpoint.isNear(m_linearIntake.getHeight(), LinearIntakeConstants.POSITION_TOLERANCE);
  }

  public Command elevCmd(double dutycycle) {
    return m_linearIntake.set(dutycycle);
  }

  public Command extend() {
    return setLinearPosition(LinearIntakeConstants.EXTENDED_POSITION);
  }

  public Command retract() {
    return setLinearPosition(LinearIntakeConstants.RETRACTED_POSITION);
  }

  public Command set(double dutycycle) {
    return m_linearIntake.set(dutycycle);
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
