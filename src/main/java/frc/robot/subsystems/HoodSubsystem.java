// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(HoodConstants.MOTOR_ID);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                    HoodConstants.PID_kP,
                    HoodConstants.PID_kI,
                    HoodConstants.PID_kD,
                    HoodConstants.MAX_VELOCITY_RPM,
                    HoodConstants.MAX_ACCELERATION_RPS2)
            .withGearing(new MechanismGearing(HoodConstants.GEARBOX))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT_AMPS)
            .withMotorInverted(false)
            .withClosedLoopRampRate(HoodConstants.CLOSED_LOOP_RAMP_RATE_SEC)
            .withOpenLoopRampRate(HoodConstants.OPEN_LOOP_RAMP_RATE_SEC)
            .withFeedforward(HoodConstants.FEEDFORWARD)
            .withSimFeedforward(HoodConstants.SIM_FEEDFORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController smartMotorController = new TalonFXWrapper(
            motor,
            HoodConstants.MOTOR,
            smcConfig);

    private final ArmConfig hoodConfig = new ArmConfig(smartMotorController)
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withSoftLimits(HoodConstants.SOFT_LIMIT_MIN, HoodConstants.SOFT_LIMIT_MAX)
            .withHardLimit(HoodConstants.HARD_LIMIT_MIN, HoodConstants.HARD_LIMIT_MAX); // The Hood can be modeled as an
                                                                                        // arm since it has a
    // gravitational force acted upon based on the angle its in

    private final Arm hood = new Arm(hoodConfig);

    public HoodSubsystem() {

    }

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    public void setAngleDirect(Angle angle) {
        smartMotorController.setPosition(angle);
    }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public Command sysId() {
        return hood.sysId(
                HoodConstants.SYSID_MAX_VOLTAGE,
                HoodConstants.SYSID_STEP,
                HoodConstants.SYSID_DURATION);
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return hood.set(dutyCycleSupplier);
    }

    public Command setDutyCycle(double dutyCycle) {
        return hood.set(dutyCycle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
