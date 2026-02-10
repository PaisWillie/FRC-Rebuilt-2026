// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  private final Servo m_motor;

  public HopperSubsystem() {
    m_motor = new Servo(HopperConstants.MOTOR_ID);
  }

  public void setPosition(double degrees) {
    m_motor.setAngle(degrees);
  }

  @Override
  public void periodic() {

  }
}
