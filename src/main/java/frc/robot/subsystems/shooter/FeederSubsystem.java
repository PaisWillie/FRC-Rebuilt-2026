// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    private final TalonFX m_motor;

    // TODO: Add beam break sensors

    public FeederSubsystem() {
        m_motor = new TalonFX(FeederConstants.MOTOR_CAN_ID);

        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        MotorOutputConfigs m_motorOutputConfig = new MotorOutputConfigs();

        // Configure current limits for motor protection.
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.MOTOR_STATOR_CURRENT_LIMIT;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.MOTOR_SUPPLY_CURRENT_LIMIT;

        // Set motor neutral behavior.
        m_motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);
        m_motorConfig.withMotorOutput(m_motorOutputConfig);

        // Apply configuration to the motor controller.
        m_motor.getConfigurator().apply(m_motorConfig);
    }

    /**
     * Sets the intake motor speed.
     *
     * @param speed the desired motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    /**
     * Stops the intake motor.
     * 
     * @return a Command that stops the feeder when executed
     */
    public Command stop() {
        return this.runOnce(() -> {
            m_motor.stopMotor();
        });
    }

    /**
     * Runs the feeder at the predefined speed.
     *
     * @return a Command that runs the feeder when executed
     */
    public Command feed() {
        return this.runOnce(() -> {
            setSpeed(FeederConstants.FEEDER_SPEED);
        });
    }

    @Override
    public void periodic() {
    }
}
