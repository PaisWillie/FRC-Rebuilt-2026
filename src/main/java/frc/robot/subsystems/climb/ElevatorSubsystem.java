// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Robot;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_leaderMotor;
    private final TalonFX m_followerMotor;

    // Generic Smart Motor Controller with our options and vendor motor.
    private final SmartMotorController m_leaderSmartMotorController;
    private final SmartMotorController m_followerSmartMotorController;

    private final SmartMotorControllerConfig m_leaderMotorSMCConfig;

    // Arm mechanism
    private final Elevator m_climb;

    public ElevatorSubsystem() {
        m_leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_CAN_ID);
        m_followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_CAN_ID);

        SmartMotorControllerConfig followerMotorSMCConfig = new SmartMotorControllerConfig(this)
                .withMotorInverted(true)
                .withIdleMode(MotorMode.BRAKE)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMechanismCircumference(
                        ElevatorConstants.CHAIN_PITCH.times(ElevatorConstants.TOOTH_COUNT))
                .withGearing(ElevatorConstants.GEARBOX)
                .withStatorCurrentLimit(ElevatorConstants.STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(ElevatorConstants.SUPPLY_CURRENT_LIMIT)
                .withClosedLoopRampRate(ElevatorConstants.CLOSED_LOOP_RAMP_RATE)
                .withOpenLoopRampRate(ElevatorConstants.OPEN_LOOP_RAMP_RATE)
                .withTelemetry("FollowerElevatorMotor", Constants.TELEMETRY_VERBOSITY)
                .withClosedLoopController(
                        ElevatorConstants.PID_kP,
                        ElevatorConstants.PID_kI,
                        ElevatorConstants.PID_kD,
                        ElevatorConstants.MAX_VELOCITY,
                        ElevatorConstants.MAX_ACCELERATION)
                .withSimClosedLoopController(
                        ElevatorConstants.SIM_PID_kP,
                        ElevatorConstants.SIM_PID_kI,
                        ElevatorConstants.SIM_PID_kD,
                        ElevatorConstants.MAX_VELOCITY,
                        ElevatorConstants.MAX_ACCELERATION)
                .withFeedforward(new ElevatorFeedforward(
                        ElevatorConstants.FEEDFORWARD_kS,
                        ElevatorConstants.FEEDFORWARD_kG,
                        ElevatorConstants.FEEDFORWARD_kV,
                        ElevatorConstants.FEEDFORWARD_kA))
                .withSoftLimit(ElevatorConstants.SOFT_LOWER_LIMIT, ElevatorConstants.SOFT_UPPER_LIMIT);

        m_followerSmartMotorController = new TalonFXWrapper(m_followerMotor, ElevatorConstants.FOLLOWER_MOTOR,
                followerMotorSMCConfig);

        m_leaderMotorSMCConfig = new SmartMotorControllerConfig(this)
                .withMotorInverted(false)
                .withIdleMode(MotorMode.BRAKE)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMechanismCircumference(
                        ElevatorConstants.CHAIN_PITCH.times(ElevatorConstants.TOOTH_COUNT))
                .withGearing(ElevatorConstants.GEARBOX)
                .withStatorCurrentLimit(ElevatorConstants.STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(ElevatorConstants.SUPPLY_CURRENT_LIMIT)
                .withClosedLoopRampRate(ElevatorConstants.CLOSED_LOOP_RAMP_RATE)
                .withOpenLoopRampRate(ElevatorConstants.OPEN_LOOP_RAMP_RATE)
                .withTelemetry("LeaderElevatorMotor", Constants.TELEMETRY_VERBOSITY)
                .withClosedLoopController(
                        ElevatorConstants.PID_kP,
                        ElevatorConstants.PID_kI,
                        ElevatorConstants.PID_kD,
                        ElevatorConstants.MAX_VELOCITY,
                        ElevatorConstants.MAX_ACCELERATION)
                .withSimClosedLoopController(
                        ElevatorConstants.SIM_PID_kP,
                        ElevatorConstants.SIM_PID_kI,
                        ElevatorConstants.SIM_PID_kD,
                        ElevatorConstants.MAX_VELOCITY,
                        ElevatorConstants.MAX_ACCELERATION)
                .withFeedforward(new ElevatorFeedforward(
                        ElevatorConstants.FEEDFORWARD_kS,
                        ElevatorConstants.FEEDFORWARD_kG,
                        ElevatorConstants.FEEDFORWARD_kV,
                        ElevatorConstants.FEEDFORWARD_kA))
                .withSoftLimit(ElevatorConstants.SOFT_LOWER_LIMIT, ElevatorConstants.SOFT_UPPER_LIMIT)
                .withLooselyCoupledFollowers(m_followerSmartMotorController);

        m_leaderSmartMotorController = new TalonFXWrapper(m_leaderMotor, ElevatorConstants.LEADER_MOTOR,
                m_leaderMotorSMCConfig);

        ElevatorConfig climbConfig = new ElevatorConfig(m_leaderSmartMotorController)
                .withMass(ElevatorConstants.MASS)
                .withTelemetry("Elevator", Constants.TELEMETRY_VERBOSITY)
                .withHardLimits(ElevatorConstants.HARD_LOWER_LIMIT, ElevatorConstants.HARD_UPPER_LIMIT);

        if (Robot.isSimulation()) {
            climbConfig.withStartingHeight(ElevatorConstants.STARTING_HEIGHT);
        }

        m_climb = new Elevator(climbConfig);
    }

    /**
     * Creates a SysId characterization command for the elevator.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_climb.sysId(
                Volts.of(6), Volts.of(1).per(Second), Second.of(15))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
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
        Debouncer currentDebouncer = new Debouncer(ElevatorConstants.HOMING_DEBOUNCE_TIME.in(Seconds));
        Voltage runVolts = ElevatorConstants.HOMING_RUN_VOLTS;
        Distance limitHit = ElevatorConstants.HARD_UPPER_LIMIT;
        AngularVelocity velocityThreshold = ElevatorConstants.HOMING_VELOCITY_THRESHOLD;
        return Commands
                .startRun(m_leaderSmartMotorController::stopClosedLoopController,
                        () -> m_leaderSmartMotorController.setVoltage(runVolts))
                .until(() -> currentDebouncer
                        .calculate(m_leaderSmartMotorController.getStatorCurrent()
                                .gte(threshold) &&
                                m_leaderSmartMotorController.getMechanismVelocity().abs(
                                        DegreesPerSecond) <= velocityThreshold
                                                .in(DegreesPerSecond)))
                .finallyDo(() -> {
                    m_leaderSmartMotorController.setEncoderPosition(limitHit);
                    m_leaderSmartMotorController.startClosedLoopController();
                });
    }

    public Command elevCmd(double dutycycle) {
        return m_climb.set(dutycycle);
    }

    public Command setHeight(Distance height) {
        return m_climb.setHeight(height);
    }

    public Distance getPosition() {
        return m_climb.getHeight();
    }

    public boolean isClimberUp() {
        return getPosition().gt(ElevatorConstants.FIRST_LEVEL_HEIGHT);
    }

    public Optional<Distance> getSetpoint() {
        Optional<Angle> angleSetpoint = m_climb.getMechanismSetpoint();
        if (!angleSetpoint.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(m_leaderMotorSMCConfig.convertFromMechanism(angleSetpoint.get()));
    }

    public Command stop() {
        return setHeight(m_climb.getHeight());
    }

    @Override
    public void periodic() {
        m_climb.updateTelemetry();

        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("Elevator/position (m)", m_climb.getHeight().in(Meters));
            SmartDashboard.putNumber("Elevator/setpoint (m)",
                    getSetpoint().map(pos -> pos.in(Meters)).orElse(Double.NaN));

        }
    }

    @Override
    public void simulationPeriodic() {
        m_climb.simIterate();
    }
}