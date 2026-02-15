// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX m_motor;

    private final SmartMotorControllerConfig m_smcConfig;

    private final SmartMotorController m_smartMotorController;

    private final FlyWheelConfig m_flywheelConfig;

    private final FlyWheel m_flywheel;

    public FlywheelSubsystem() {
        m_motor = new TalonFX(FlywheelConstants.MOTOR_CAN_ID);

        m_smcConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(
                        FlywheelConstants.PID_kP,
                        FlywheelConstants.PID_kI,
                        FlywheelConstants.PID_kD,
                        FlywheelConstants.MAX_VELOCITY_RPM,
                        FlywheelConstants.MAX_ACCELERATION_RPS2)
                .withGearing(new MechanismGearing(FlywheelConstants.GEARBOX))
                .withIdleMode(MotorMode.COAST) // Keep spinning even if not powered
                .withTelemetry("FlywheelMotor", Constants.TELEMETRY_VERBOSITY)
                .withStatorCurrentLimit(FlywheelConstants.STATOR_CURRENT_LIMIT_AMPS)
                .withMotorInverted(false)
                .withClosedLoopRampRate(FlywheelConstants.CLOSED_LOOP_RAMP_RATE_SEC)
                .withOpenLoopRampRate(FlywheelConstants.OPEN_LOOP_RAMP_RATE_SEC)
                .withFeedforward(FlywheelConstants.FEEDFORWARD)
                .withSimFeedforward(FlywheelConstants.SIM_FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_smartMotorController = new TalonFXWrapper(
                m_motor,
                FlywheelConstants.MOTOR,
                m_smcConfig);

        m_flywheelConfig = new FlyWheelConfig(m_smartMotorController)
                .withDiameter(FlywheelConstants.DIAMETER_INCHES)
                .withMass(FlywheelConstants.MASS_POUNDS)
                .withTelemetry("FlywheelMech", Constants.TELEMETRY_VERBOSITY)
                .withSoftLimit(RPM.of(-FlywheelConstants.SOFT_LIMIT_RPM), RPM.of(FlywheelConstants.SOFT_LIMIT_RPM))
                .withSpeedometerSimulation(FlywheelConstants.SIM_MAX_VELOCITY_RPM);

        m_flywheel = new FlyWheel(m_flywheelConfig);
    }

    /**
     * Gets the current flywheel velocity.
     *
     * @return the current angular velocity
     */
    public AngularVelocity getVelocity() {
        return m_flywheel.getSpeed();
    }

    /**
     * Creates a command to set the flywheel velocity.
     *
     * @param speed the target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setVelocity(AngularVelocity speed) {
        return m_flywheel.setSpeed(speed);
    }

    /**
     * Creates a command to set the flywheel velocity from a supplier.
     *
     * @param speed the supplier of target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return m_flywheel.setSpeed(speed);
    }

    /**
     * Creates a SysId characterization command for the flywheel.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5)); // TODO
    }

    /**
     * Runs the flywheel simulation step.
     */
    @Override
    public void simulationPeriodic() {
        m_flywheel.simIterate();
    }

    /**
     * Creates a command to set the flywheel speed based on linear velocity.
     *
     * @param speed the desired linear velocity
     * @return the command that sets flywheel speed
     */
    public Command setRPM(LinearVelocity speed) {
        return m_flywheel.setSpeed(RotationsPerSecond
                .of(speed.in(MetersPerSecond) / FlywheelConstants.DIAMETER_INCHES.times(Math.PI).in(Meters)));
    }

    /**
     * Directly sets flywheel speed based on linear velocity.
     *
     * @param speed the desired linear velocity
     */
    public void setRPMDirect(LinearVelocity speed) {
        m_smartMotorController.setVelocity(RotationsPerSecond
                .of(speed.in(MetersPerSecond) / FlywheelConstants.DIAMETER_INCHES.times(Math.PI).in(Meters)));
    }

    public Command shoot() {
        return setVelocity(FlywheelConstants.SHOOTING_VELOCITY_RPM);
    }

    public Command setDefaultRPM() {
        return setVelocity(FlywheelConstants.DEFAULT_VELOCITY_RPM);
    }

    public boolean isAtTargetRPM() {
        Optional<AngularVelocity> setpoint = m_smartMotorController.getMechanismSetpointVelocity();

        if (!setpoint.isPresent())
            return false;

        double deltaRPM = setpoint.get().in(RPM) - getVelocity().in(RPM);
        return Math.abs(deltaRPM) <= FlywheelConstants.RPM_TOLERANCE.in(RPM);
    }

    // TODO: Delete this
    public Command test() {
        return runOnce(() -> System.out.println("Running!"));
    }

    public Command stop() {
        return setVelocity(RPM.of(0));
    }

    /**
     * Updates flywheel telemetry.
     */
    @Override
    public void periodic() {
        m_flywheel.updateTelemetry();
    }
}
