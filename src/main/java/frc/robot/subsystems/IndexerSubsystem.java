// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final MotorOutputConfigs m_motorOutputConfig;

    public IndexerSubsystem() {
        m_motor = new TalonFX(IndexerConstants.MOTOR_ID);
        m_motorConfig = new TalonFXConfiguration();
        m_motorOutputConfig = new MotorOutputConfigs();

        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.MOTOR_SUPPLY_CURRENT_LIMIT;

        m_motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);
        m_motorConfig.withMotorOutput(m_motorOutputConfig);

        m_motor.getConfigurator().apply(m_motorConfig);
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {

    }
}
