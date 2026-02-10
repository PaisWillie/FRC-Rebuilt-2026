// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Subsystem for controlling the intake motor.
 */
public class IntakeSubsystem extends SubsystemBase {

    /**
     * PWM motor controller for the intake.
     */
    private final PWMTalonFX m_rollerMotor;

    private final TalonFX m_linearMotor;
    private final SmartMotorControllerConfig m_linearMotorSMCConfig;
    private final SmartMotorController m_linearMotorSMC;

    private final MechanismPositionConfig m_robotToMechanism;
    private ElevatorConfig m_linearConfig;
    private final Elevator m_linearIntake;

    /**
     * Constructs the intake subsystem and initializes the motor controller.
     */
    public IntakeSubsystem() {

        m_rollerMotor = new PWMTalonFX(IntakeConstants.ROLLER_MOTOR_PWM_ID);
        m_linearMotor = new TalonFX(IntakeConstants.LinearConstants.LINEAR_MOTOR_CAN_ID);

        m_linearMotorSMCConfig = new SmartMotorControllerConfig(this)
                .withMechanismCircumference(IntakeConstants.LinearConstants.MOTOR_CIRCUMFERENCE)
                .withClosedLoopController(
                        IntakeConstants.LinearConstants.PID_kP,
                        IntakeConstants.LinearConstants.PID_kI,
                        IntakeConstants.LinearConstants.PID_kD,
                        IntakeConstants.LinearConstants.MAX_VELOCITY,
                        IntakeConstants.LinearConstants.MAX_ACCELERATION)
                .withSoftLimit(
                        IntakeConstants.LinearConstants.SOFT_LIMIT_MIN,
                        IntakeConstants.LinearConstants.SOFT_LIMIT_MAX)
                .withGearing(new MechanismGearing(IntakeConstants.LinearConstants.GEARBOX))
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(IntakeConstants.LinearConstants.STATOR_CURRENT_LIMIT)
                .withMotorInverted(false)
                .withClosedLoopRampRate(IntakeConstants.LinearConstants.CLOSED_LOOP_RAMP_RATE)
                .withOpenLoopRampRate(IntakeConstants.LinearConstants.OPEN_LOOP_RAMP_RATE)
                .withFeedforward(IntakeConstants.LinearConstants.FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_linearMotorSMC = new TalonFXWrapper(
                m_linearMotor,
                IntakeConstants.LinearConstants.MOTOR,
                m_linearMotorSMCConfig);

        m_robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(IntakeConstants.LinearConstants.ROBOT_MAX_HEIGHT)
                .withMaxRobotLength(IntakeConstants.LinearConstants.ROBOT_MAX_LENGTH)
                .withRelativePosition(IntakeConstants.LinearConstants.RELATIVE_POSITION);

        m_linearConfig = new ElevatorConfig(m_linearMotorSMC)
                .withStartingHeight(IntakeConstants.LinearConstants.STARTING_HEIGHT)
                .withHardLimits(
                        IntakeConstants.LinearConstants.HARD_LIMIT_MIN,
                        IntakeConstants.LinearConstants.HARD_LIMIT_MAX)
                .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
                .withMechanismPositionConfig(m_robotToMechanism)
                .withMass(IntakeConstants.LinearConstants.MECHANISM_MASS);

        m_linearIntake = new Elevator(m_linearConfig);
    }

    /**
     * Sets the intake motor speed.
     *
     * @param speed the desired motor output (-1.0 to 1.0)
     */
    public void setRollerSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    /**
     * Stops the intake motor.
     */
    public void stopRollers() {
        m_rollerMotor.stopMotor();
    }

    /**
     * Creates a command to set the linear intake height.
     *
     * @param height the desired height
     * @return the command that sets the height
     */
    public Command setLinearPosition(Distance height) {
        return m_linearIntake.setHeight(height);
    }

    /**
     * Updates intake telemetry.
     */
    @Override
    public void periodic() {
        m_linearIntake.updateTelemetry();
    }

    /**
     * Runs the intake simulation step.
     */
    @Override
    public void simulationPeriodic() {
        m_linearIntake.simIterate();
    }
}
