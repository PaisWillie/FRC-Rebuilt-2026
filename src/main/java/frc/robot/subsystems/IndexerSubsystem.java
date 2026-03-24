// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * Subsystem for controlling the indexer motor and its configuration.
 */
public class IndexerSubsystem extends SubsystemBase {

    /** TalonFX motor controller for the indexer. */
    private final TalonFX m_motor;
    /** Base configuration for the motor controller. */
    private final TalonFXConfiguration m_motorConfig;
    /** Output configuration for neutral mode and output behavior. */
    private final MotorOutputConfigs m_motorOutputConfig;

    private double m_targetSpeed = Double.NaN;

    /**
     * Constructs the indexer subsystem and applies motor configuration.
     */
    public IndexerSubsystem() {
        m_motor = new TalonFX(IndexerConstants.MOTOR_CAN_ID);
        m_motorConfig = new TalonFXConfiguration();
        m_motorOutputConfig = new MotorOutputConfigs();

        // Configure current limits for motor protection.
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.MOTOR_SUPPLY_CURRENT_LIMIT;

        // Set motor neutral behavior.
        m_motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);
        m_motorConfig.withMotorOutput(m_motorOutputConfig);

        // Apply configuration to the motor controller.
        m_motor.getConfigurator().apply(m_motorConfig);
    }

    /**
     * Sets the indexer motor speed.
     *
     * @param speed the desired motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        if (speed != m_targetSpeed) {
            m_targetSpeed = speed;
            m_motor.set(speed);
        }
    }

    /**
     * Stops the indexer motor.
     */
    public Command stop() {
        return this.runOnce(() -> m_motor.stopMotor());
    }

    // Helper method to alternate between full and half speed every 0.125 seconds
    private boolean useFullSpeed() {
        double secondFraction = Timer.getFPGATimestamp() % 2.0;
        return secondFraction >= 0.125;
    }

    // TODO: Understand why alternating between two constants doesn't pulse in
    // simulation, but alternating between a constant and zero does.
    public Command run() {
        return new ConditionalCommand(
                this.runOnce(() -> setSpeed(IndexerConstants.INDEXER_FULL_SPEED)),
                this.runOnce(() -> setSpeed(0)),
                this::useFullSpeed).repeatedly();
    }

    public Command reverse() {
        return new ConditionalCommand(
                this.runOnce(() -> setSpeed(-IndexerConstants.INDEXER_FULL_SPEED)),
                this.runOnce(() -> setSpeed(0)),
                this::useFullSpeed).repeatedly();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
