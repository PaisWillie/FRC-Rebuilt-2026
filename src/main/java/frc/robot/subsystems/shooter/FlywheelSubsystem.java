// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final TalonFX m_leaderMotor;
    private final TalonFX m_followerMotor;

    private final SmartMotorController m_smartMotorController;

    private final FlyWheel m_flywheel;

    private final Debouncer m_atRPMDebouncer = new Debouncer(FlywheelConstants.AT_RPM_DEBOUNCE_TIME,
            Debouncer.DebounceType.kRising);

    public FlywheelSubsystem() {
        m_leaderMotor = new TalonFX(FlywheelConstants.LEADER_MOTOR_CAN_ID);
        m_followerMotor = new TalonFX(FlywheelConstants.FOLLOWER_MOTOR_CAN_ID);

        SmartMotorControllerConfig m_smcConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(
                        FlywheelConstants.PID_kP,
                        FlywheelConstants.PID_kI,
                        FlywheelConstants.PID_kD,
                        FlywheelConstants.MAX_VELOCITY_RPM,
                        FlywheelConstants.MAX_ACCELERATION_RPS2)
                .withSimClosedLoopController(
                        FlywheelConstants.SIM_PID_kP,
                        FlywheelConstants.SIM_PID_kI,
                        FlywheelConstants.SIM_PID_kD,
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
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withFollowers(Pair.of(m_followerMotor, true));

        m_smartMotorController = new TalonFXWrapper(
                m_leaderMotor,
                FlywheelConstants.MOTOR,
                m_smcConfig);

        FlyWheelConfig m_flywheelConfig = new FlyWheelConfig(m_smartMotorController)
                .withDiameter(FlywheelConstants.DIAMETER_INCHES)
                .withMass(FlywheelConstants.MASS_POUNDS)
                .withTelemetry("FlywheelMech", Constants.TELEMETRY_VERBOSITY)
                .withSoftLimit(RPM.of(-FlywheelConstants.SOFT_LIMIT_RPM), RPM.of(FlywheelConstants.SOFT_LIMIT_RPM))
                .withSpeedometerSimulation(FlywheelConstants.SIM_MAX_VELOCITY_RPM);

        m_flywheel = new FlyWheel(m_flywheelConfig);
    }

    /**
     * Creates a SysId characterization command for the flywheel.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_flywheel.sysId(
                Volts.of(10), Volts.of(1).per(Second), Second.of(5))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
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

    public Optional<AngularVelocity> getSetpointVelocity() {
        Optional<AngularVelocity> setpoint = m_smartMotorController.getMechanismSetpointVelocity();

        if (!setpoint.isPresent())
            return Optional.empty();

        // Convert from output to input velocity for comparison with actual velocity
        return Optional.of(setpoint.get().times(FlywheelConstants.GEARBOX.getInputToOutputConversionFactor()));
    }

    public boolean isAtTargetRPM() {
        Optional<AngularVelocity> setpoint = getSetpointVelocity();

        if (!setpoint.isPresent())
            return false;

        return m_atRPMDebouncer.calculate(
                setpoint.get().isNear(getVelocity(), FlywheelConstants.RPM_TOLERANCE));
    }

    public Command stop() {
        return runOnce(() -> {
            m_smartMotorController.setVoltage(Volts.of(0));
        });
    }

    /**
     * Updates flywheel telemetry.
     */
    @Override
    public void periodic() {
        m_flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // Run the flywheel simulation step
        m_flywheel.simIterate();

        SmartDashboard.putNumber("Flywheel Velocity (RPM)", getVelocity().in(RPM));
        SmartDashboard.putNumber("Flywheel Setpoint Velocity (RPM)",
                getSetpointVelocity().orElse(RotationsPerSecond.of(0)).in(RPM)
                        * FlywheelConstants.GEARBOX.getOutputToInputConversionFactor());
    }
}
