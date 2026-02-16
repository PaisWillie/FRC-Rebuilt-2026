// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ClimbConstants.LEADER_MOTOR_CAN_ID);
    private final Distance m_circumference = ClimbConstants.CHAIN_PITCH.times(ClimbConstants.TOOTH_COUNT);
    private final Distance m_radius = m_circumference.div(2 * Math.PI);
    /*
     * Using a measuring tape, where 0 cm marks the elevator at its lowest point,
     * you can measure the height to determine the starting position reference.
     */
    private final Distance m_startingHeight = ClimbConstants.STARTING_HEIGHT;
    /*
     * This is the STARTING PID Controller for the Elevator. If you are using a
     * TalonFX or TalonFXS this will run on the motor controller itself.
     */
    private final ExponentialProfilePIDController m_pidController = new ExponentialProfilePIDController(
            ClimbConstants.PID_kP,
            ClimbConstants.PID_kI,
            ClimbConstants.PID_kD,
            ExponentialProfilePIDController.createElevatorConstraints(
                    Volts.of(12),
                    ClimbConstants.MOTOR,
                    ClimbConstants.MASS,
                    m_radius,
                    ClimbConstants.GEARBOX));

    private final ExponentialProfilePIDController m_simPidController = new ExponentialProfilePIDController(
            ClimbConstants.SIM_PID_kP,
            ClimbConstants.SIM_PID_kI,
            ClimbConstants.SIM_PID_kD,
            ExponentialProfilePIDController.createElevatorConstraints(
                    Volts.of(12),
                    ClimbConstants.MOTOR,
                    ClimbConstants.MASS,
                    m_radius,
                    ClimbConstants.GEARBOX));
    /*
     * This is the STARTING Feedforward for the Elevator. If you are using a TalonFX
     * or TalonFXS this will run on the motor controller itself.
     */
    private final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(
            ClimbConstants.FEEDFORWARD_kS,
            ClimbConstants.FEEDFORWARD_kG,
            ClimbConstants.FEEDFORWARD_kV,
            ClimbConstants.FEEDFORWARD_kA);
    /**
     * {@link SmartMotorControllerConfig} for the elevator motor.
     */
    private final SmartMotorControllerConfig m_leaderMotorConfig = new SmartMotorControllerConfig(this)
            /*
             * Basic Configuration options for the motor
             */
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMechanismCircumference(m_circumference)
            .withGearing(ClimbConstants.GEARBOX)
            .withStatorCurrentLimit(ClimbConstants.STATOR_CURRENT_LIMIT)
            .withClosedLoopRampRate(ClimbConstants.CLOSED_LOOP_RAMP_RATE)
            .withOpenLoopRampRate(ClimbConstants.OPEN_LOOP_RAMP_RATE)
            .withTelemetry("ExponentiallyProfiledElevatorMotor", Constants.TELEMETRY_VERBOSITY) // Could have more
                                                                                                // fine-grained
            // control over what gets
            // reported with
            // SmartMotorControllerTelemetryConfig
            /*
             * Closed loop configuration options for the motor.
             */
            .withClosedLoopController(m_pidController)
            .withSimClosedLoopController(m_simPidController)
            .withFeedforward(m_elevatorFeedforward)
            .withSoftLimit(ClimbConstants.SOFT_LOWER_LIMIT, ClimbConstants.SOFT_UPPER_LIMIT)
            .withFollowers(Pair.of(new TalonFX(ClimbConstants.FOLLOWER_MOTOR_CAN_ID), true));

    /// Generic Smart Motor Controller with our options and vendor motor.
    private final SmartMotorController m_smartMotorController = new TalonFXWrapper(m_motor, ClimbConstants.MOTOR,
            m_leaderMotorConfig);

    /// Elevator-specific options
    private ElevatorConfig m_climbConfig = new ElevatorConfig(m_smartMotorController)
            /*
             * Basic configuration options for the arm.
             */
            .withMass(ClimbConstants.MASS)
            .withStartingHeight(m_startingHeight) // The starting position should ONLY be defined if you are NOT using
                                                  // an
                                                  // absolute encoder.
            .withTelemetry("ExponentiallyProfiledElevator", Constants.TELEMETRY_VERBOSITY)
            /*
             * Simulation configuration options for the arm.
             */
            .withHardLimits(ClimbConstants.HARD_LOWER_LIMIT, ClimbConstants.HARD_UPPER_LIMIT);
    // Arm mechanism
    private final Elevator m_climb = new Elevator(m_climbConfig);

    public ClimbSubsystem() {
    }

    /**
     * Reset the encoder to the lowest position when the current threshhold is
     * reached. Should be used when the Elevator
     * position is unreliable, like startup. Threshhold is only detected if exceeded
     * for 0.4 seconds, and the motor moves
     * less than 2 degrees per second.
     *
     * @param threshold The current threshhold held when the Elevator is at it's
     *                  hard limit.
     * @return
     */
    public Command homing(Current threshold) {
        Debouncer currentDebouncer = new Debouncer(ClimbConstants.HOMING_DEBOUNCE_TIME.in(Seconds));
        Voltage runVolts = ClimbConstants.HOMING_RUN_VOLTS;
        Distance limitHit = ClimbConstants.HARD_UPPER_LIMIT;
        AngularVelocity velocityThreshold = ClimbConstants.HOMING_VELOCITY_THRESHOLD;
        return Commands
                .startRun(m_smartMotorController::stopClosedLoopController,
                        () -> m_smartMotorController.setVoltage(runVolts))
                .until(() -> currentDebouncer.calculate(m_smartMotorController.getStatorCurrent().gte(threshold) &&
                        m_smartMotorController.getMechanismVelocity().abs(DegreesPerSecond) <= velocityThreshold
                                .in(DegreesPerSecond)))
                .finallyDo(() -> {
                    m_smartMotorController.setEncoderPosition(limitHit);
                    m_smartMotorController.startClosedLoopController();
                });
    }

    public Command elevCmd(double dutycycle) {
        return m_climb.set(dutycycle);
    }

    public Command setHeight(Distance height) {
        return m_climb.setHeight(height);
    }

    public Command sysId() {
        return m_climb.sysId(ClimbConstants.SYSID_MAX_VOLTAGE, ClimbConstants.SYSID_STEP,
                ClimbConstants.SYSID_DURATION);
    }

    public Command test() {
        return runOnce(() -> System.out.println("Running!"));
    }

    @Override
    public void periodic() {
        m_climb.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_climb.simIterate();
    }
}
