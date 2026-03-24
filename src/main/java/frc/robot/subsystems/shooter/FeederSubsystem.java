// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.FeetPerSecond;
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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FeederConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FeederSubsystem extends SubsystemBase {

    private final TalonFX m_motor;

    private final SmartMotorController m_smartMotorController;

    private final FlyWheel m_feeder;

    private final DigitalInput m_beamBreak;

    public FeederSubsystem() {
        m_motor = new TalonFX(FeederConstants.MOTOR_CAN_ID);
        m_beamBreak = new DigitalInput(FeederConstants.BEAM_BREAK_DIO_PORT);

        SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(
                        FeederConstants.PID_kP,
                        FeederConstants.PID_kI,
                        FeederConstants.PID_kD,
                        FeederConstants.MAX_VELOCITY_RPM,
                        FeederConstants.MAX_ACCELERATION_RPS2)
                .withSimClosedLoopController(
                        FeederConstants.SIM_PID_kP,
                        FeederConstants.SIM_PID_kI,
                        FeederConstants.SIM_PID_kD,
                        FeederConstants.MAX_VELOCITY_RPM,
                        FeederConstants.MAX_ACCELERATION_RPS2)
                .withGearing(new MechanismGearing(FeederConstants.GEARBOX))
                .withIdleMode(FeederConstants.IDLE_MODE) // Keep spinning even if not powered
                .withTelemetry("FeederMotor", Constants.TELEMETRY_VERBOSITY)
                .withStatorCurrentLimit(FeederConstants.STATOR_CURRENT_LIMIT_AMPS)
                .withMotorInverted(FeederConstants.MOTOR_INVERTED)
                .withClosedLoopRampRate(FeederConstants.CLOSED_LOOP_RAMP_RATE_SEC)
                .withOpenLoopRampRate(FeederConstants.OPEN_LOOP_RAMP_RATE_SEC)
                .withFeedforward(FeederConstants.FEEDFORWARD)
                .withSimFeedforward(FeederConstants.SIM_FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMomentOfInertia(FeederConstants.MOI);

        m_smartMotorController = new TalonFXWrapper(
                m_motor,
                FeederConstants.MOTOR,
                smcConfig);

        FlyWheelConfig feederConfig = new FlyWheelConfig(m_smartMotorController)
                .withDiameter(FeederConstants.DIAMETER_INCHES)
                .withMOI(FeederConstants.MOI)
                .withTelemetry("FeederMech", Constants.TELEMETRY_VERBOSITY)
                .withSoftLimit(FeederConstants.SOFT_LIMIT_RPM.times(-1), FeederConstants.SOFT_LIMIT_RPM)
                .withSpeedometerSimulation(FeederConstants.SIM_MAX_VELOCITY_RPM);

        m_feeder = new FlyWheel(feederConfig);
    }

    /**
     * Creates a SysId characterization command for the flywheel.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_feeder.sysId(
                Volts.of(12), Volts.of(3).per(Second), Second.of(10))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
    }

    /**
     * Gets the current flywheel velocity.
     *
     * @return the current angular velocity
     */
    public AngularVelocity getAngularVelocity() {
        return m_feeder.getSpeed();
    }

    /**
     * Gets the current flywheel velocity.
     * 
     * @return the current linear velocity
     */
    public LinearVelocity getLinearVelocity() {
        return m_feeder.getLinearVelocity();
    }

    /**
     * Creates a command to set the flywheel velocity.
     *
     * @param speed the target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(AngularVelocity speed) {
        return m_feeder.setSpeed(speed);
    }

    /**
     * Creates a command to set the flywheel velocity from a supplier.
     *
     * @param speed the supplier of target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(Supplier<AngularVelocity> speed) {
        return m_feeder.setSpeed(speed);
    }

    /**
     * Creates a command to set the flywheel speed based on linear velocity.
     *
     * @param speed the desired linear velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(LinearVelocity speed) {
        return m_feeder.setSpeed(RotationsPerSecond
                .of(speed.in(MetersPerSecond) / FeederConstants.DIAMETER_INCHES.times(Math.PI).in(Meters)));
    }

    public Optional<AngularVelocity> getSetpointVelocity() {
        Optional<AngularVelocity> setpoint = m_smartMotorController.getMechanismSetpointVelocity();

        if (!setpoint.isPresent())
            return Optional.empty();

        // Convert from output to input velocity for comparison with actual velocity
        return Optional.of(setpoint.get().times(FeederConstants.GEARBOX.getInputToOutputConversionFactor()));
    }

    public Command feed() {
        return setSpeed(FeederConstants.FEEDER_SPEED);
    }

    public Command reverse() {
        return setSpeed(FeederConstants.REVERSE_SPEED);
    }

    public Command stop() {
        return this.runOnce(() -> {
            m_smartMotorController.stopClosedLoopController();
            m_smartMotorController.setDutyCycle(0);
        });
    }

    /**
     * Checks if the beam break sensor is triggered, indicating that a ball is
     * present in the feeder.
     * 
     * @return true if the beam is broken, false otherwise
     */
    public boolean isBeamBroken() {
        return m_beamBreak.get();
    }

    /**
     * Updates flywheel telemetry.
     */
    @Override
    public void periodic() {
        m_feeder.updateTelemetry();

        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("FeederMech/linearVelocity (fps)", getLinearVelocity().in(FeetPerSecond));
            SmartDashboard.putNumber("FeederMech/velocity (RPM)", getAngularVelocity().in(RPM));
            SmartDashboard.putNumber("FeederMech/setpoint (RPM)",
                    getSetpointVelocity().map(
                            setpoint -> setpoint.in(RPM) * FeederConstants.GEARBOX.getOutputToInputConversionFactor())
                            .orElse(Double.NaN));
        }
    }

    @Override
    public void simulationPeriodic() {
        m_feeder.simIterate();
    }
}