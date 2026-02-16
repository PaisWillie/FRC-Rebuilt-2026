// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants.TongueConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TongueSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SmartMotorController m_smartMotorController;
    private final Elevator m_elevator;

    public TongueSubsystem() {
        m_motor = new SparkMax(TongueConstants.MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                .withMechanismCircumference(TongueConstants.MECHANISM_CIRCUMFERENCE)
                .withClosedLoopController(
                        TongueConstants.PID_kP,
                        TongueConstants.PID_kI,
                        TongueConstants.PID_kD,
                        TongueConstants.MAX_VELOCITY,
                        TongueConstants.MAX_ACCELERATION)
                .withSoftLimit(TongueConstants.SOFT_LIMIT_MIN, TongueConstants.SOFT_LIMIT_MAX)
                .withGearing(new MechanismGearing(TongueConstants.GEARBOX))
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(TongueConstants.STATOR_CURRENT_LIMIT)
                .withMotorInverted(false)
                .withClosedLoopRampRate(TongueConstants.CLOSED_LOOP_RAMP_RATE)
                .withOpenLoopRampRate(TongueConstants.OPEN_LOOP_RAMP_RATE)
                .withFeedforward(TongueConstants.FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_smartMotorController = new SparkWrapper(m_motor, DCMotor.getNEO(1), motorConfig);

        MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(TongueConstants.ROBOT_MAX_HEIGHT)
                .withMaxRobotLength(TongueConstants.ROBOT_MAX_LENGTH)
                .withRelativePosition(TongueConstants.RELATIVE_POSITION);

        ElevatorConfig config = new ElevatorConfig(m_smartMotorController)
                .withStartingHeight(TongueConstants.STARTING_HEIGHT)
                .withHardLimits(TongueConstants.HARD_LIMIT_MIN, TongueConstants.HARD_LIMIT_MAX)
                .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
                .withMechanismPositionConfig(robotToMechanism)
                .withMass(TongueConstants.MECHANISM_MASS);

        m_elevator = new Elevator(config);
    }

    /**
     * Creates a SysId characterization command for the tongue.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_elevator.sysId(
                Volts.of(12), Volts.of(12).per(Second), Second.of(30))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
    }

    public Command elevCmd(double dutycycle) {
        return m_elevator.set(dutycycle);
    }

    public Command setDistance(Distance distance) {
        return m_elevator.setHeight(distance);
    }

    @Override
    public void periodic() {
        m_elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_elevator.simIterate();
    }
}
