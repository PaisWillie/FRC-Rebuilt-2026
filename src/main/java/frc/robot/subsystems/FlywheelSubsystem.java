// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private final Distance m_diameter = FlywheelConstants.DIAMETER_INCHES;
    private final TalonFX m_motor = new TalonFX(FlywheelConstants.MOTOR_ID);

    private final SmartMotorControllerConfig m_smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                    FlywheelConstants.PID_kP,
                    FlywheelConstants.PID_kI,
                    FlywheelConstants.PID_kD,
                    FlywheelConstants.MAX_VELOCITY_RPM,
                    FlywheelConstants.MAX_ACCELERATION_RPS2)
            .withGearing(new MechanismGearing(FlywheelConstants.GEARBOX))
            .withIdleMode(MotorMode.COAST) // Keep spinning even if not powered
            .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(FlywheelConstants.STATOR_CURRENT_LIMIT_AMPS)
            .withMotorInverted(false)
            .withClosedLoopRampRate(FlywheelConstants.CLOSED_LOOP_RAMP_RATE_SEC)
            .withOpenLoopRampRate(FlywheelConstants.OPEN_LOOP_RAMP_RATE_SEC)
            .withFeedforward(FlywheelConstants.FEEDFORWARD)
            .withSimFeedforward(FlywheelConstants.SIM_FEEDFORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController m_smartMotorController = new TalonFXWrapper(
            m_motor,
            FlywheelConstants.MOTOR,
            m_smcConfig);

    private final FlyWheelConfig m_flywheelConfig = new FlyWheelConfig(m_smartMotorController)
            .withDiameter(FlywheelConstants.DIAMETER_INCHES)
            .withMass(FlywheelConstants.MASS_POUNDS)
            .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
            .withSoftLimit(RPM.of(-FlywheelConstants.SOFT_LIMIT_RPM), RPM.of(FlywheelConstants.SOFT_LIMIT_RPM))
            .withSpeedometerSimulation(FlywheelConstants.SIM_MAX_VELOCITY_RPM);

    private final FlyWheel m_flywheel = new FlyWheel(m_flywheelConfig);

    public FlywheelSubsystem() {
    }

    public AngularVelocity getVelocity() {
        return m_flywheel.getSpeed();
    }

    public Command setVelocity(AngularVelocity speed) {
        return m_flywheel.setSpeed(speed);
    }

    public Command setDutyCycle(double dutyCycle) {
        return m_flywheel.set(dutyCycle);
    }

    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return m_flywheel.setSpeed(speed);
    }

    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return m_flywheel.set(dutyCycle);
    }

    public Command sysId() {
        return m_flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5)); // TODO
    }

    @Override
    public void simulationPeriodic() {
        m_flywheel.simIterate();
    }

    public Command setRPM(LinearVelocity speed) {
        return m_flywheel
                .setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / m_diameter.times(Math.PI).in(Meters)));
    }

    public void setRPMDirect(LinearVelocity speed) {
        m_smartMotorController
                .setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / m_diameter.times(Math.PI).in(Meters)));
    }

    @Override
    public void periodic() {
        m_flywheel.updateTelemetry();
    }
}
