// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants.LinearConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
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

    private final SmartMotorController m_linearMotorSMC;
    private final SmartMotorControllerConfig m_linearMotorSMCConfig;

    private final Elevator m_linearIntake;

    /**
     * Constructs the intake subsystem and initializes the motor controller.
     */
    public IntakeSubsystem() {

        m_rollerMotor = new PWMTalonFX(IntakeConstants.ROLLER_MOTOR_PWM_ID);
        m_linearMotor = new TalonFX(IntakeConstants.LinearConstants.LINEAR_MOTOR_CAN_ID);

        m_linearMotorSMCConfig = new SmartMotorControllerConfig(this)
                .withMechanismCircumference(IntakeConstants.LinearConstants.MOTOR_CIRCUMFERENCE_METERS)
                .withClosedLoopController(
                        IntakeConstants.LinearConstants.PID_kP,
                        IntakeConstants.LinearConstants.PID_kI,
                        IntakeConstants.LinearConstants.PID_kD,
                        IntakeConstants.LinearConstants.MAX_VELOCITY_MPS,
                        IntakeConstants.LinearConstants.MAX_ACCELERATION_MPS2)
                .withSimClosedLoopController(
                        IntakeConstants.LinearConstants.SIM_PID_kP,
                        IntakeConstants.LinearConstants.SIM_PID_kI,
                        IntakeConstants.LinearConstants.SIM_PID_kD,
                        IntakeConstants.LinearConstants.MAX_VELOCITY_MPS,
                        IntakeConstants.LinearConstants.MAX_ACCELERATION_MPS2)
                .withSoftLimit(
                        IntakeConstants.LinearConstants.SOFT_LIMIT_MIN_METERS,
                        IntakeConstants.LinearConstants.SOFT_LIMIT_MAX_METERS)
                .withGearing(new MechanismGearing(IntakeConstants.LinearConstants.GEARBOX))
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("LinearIntakeMotor", Constants.TELEMETRY_VERBOSITY)
                .withStatorCurrentLimit(IntakeConstants.LinearConstants.STATOR_CURRENT_LIMIT_AMPS)
                .withMotorInverted(false)
                .withClosedLoopRampRate(IntakeConstants.LinearConstants.CLOSED_LOOP_RAMP_RATE_SEC)
                .withOpenLoopRampRate(IntakeConstants.LinearConstants.OPEN_LOOP_RAMP_RATE_SEC)
                .withFeedforward(IntakeConstants.LinearConstants.FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_linearMotorSMC = new TalonFXWrapper(
                m_linearMotor,
                IntakeConstants.LinearConstants.MOTOR,
                m_linearMotorSMCConfig);

        MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(IntakeConstants.LinearConstants.ROBOT_MAX_HEIGHT_METERS)
                .withMaxRobotLength(IntakeConstants.LinearConstants.ROBOT_MAX_LENGTH_METERS)
                .withRelativePosition(IntakeConstants.LinearConstants.RELATIVE_POSITION_METERS);

        ElevatorConfig m_linearConfig = new ElevatorConfig(m_linearMotorSMC)
                .withStartingHeight(IntakeConstants.LinearConstants.STARTING_HEIGHT_METERS)
                .withHardLimits(
                        IntakeConstants.LinearConstants.HARD_LIMIT_MIN_METERS,
                        IntakeConstants.LinearConstants.HARD_LIMIT_MAX_METERS)
                .withTelemetry("LinearIntake", Constants.TELEMETRY_VERBOSITY)
                .withMechanismPositionConfig(m_robotToMechanism)
                .withMass(IntakeConstants.LinearConstants.MECHANISM_MASS_POUNDS);

        m_linearIntake = new Elevator(m_linearConfig);
    }

    /**
     * Creates a SysId characterization command for the linear intake.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_linearIntake.sysId(
                Volts.of(12), Volts.of(12).per(Second), Second.of(30))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
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
    public Command stopRollers() {
        return runOnce(() -> m_rollerMotor.stopMotor());
    }

    /**
     * Creates a command to set the linear intake height.
     *
     * @param position the desired position between the soft limits
     * @return the command that sets the height
     */
    public Command setLinearPosition(Distance position) {
        return m_linearIntake.setHeight(position);
    }

    public boolean isLinearAtTargetPosition() {
        Optional<Angle> angle_setpoint = m_linearIntake.getMechanismSetpoint();

        if (!angle_setpoint.isPresent())
            return false;

        Distance setpoint = m_linearMotorSMCConfig.convertFromMechanism(angle_setpoint.get());

        // TODO: Do we need a debouncer?
        return setpoint.isNear(m_linearIntake.getHeight(), IntakeConstants.LinearConstants.POSITION_TOLERANCE);
    }

    }

    public Command elevCmd(double dutycycle) {
        return m_linearIntake.set(dutycycle);
    }

    public Command intake() {
        return this.runOnce(() -> {
            setRollerSpeed(IntakeConstants.ROLLER_SPEED);
        });
    }

    public Command outtake() {
        return this.runOnce(() -> {
            setRollerSpeed(-IntakeConstants.ROLLER_SPEED);
        });
    }

    public Command extend() {
        return setLinearPosition(IntakeConstants.LinearConstants.EXTENDED_POSITION);
    }

    public Command retract() {
        return setLinearPosition(IntakeConstants.LinearConstants.RETRACTED_POSITION);
    }

    /**
     * Move the elevator up and down.
     * 
     * @param dutycycle [-1, 1] speed to set the elevator too.
     */
    public Command set(double dutycycle) {
        return m_linearIntake.set(dutycycle);
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
