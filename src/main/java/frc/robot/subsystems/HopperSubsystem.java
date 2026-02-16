// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  private final Servo m_leftMotor;
  private final Servo m_rightMotor;

  public HopperSubsystem() {
    m_leftMotor = new Servo(HopperConstants.MOTOR_PWM_ID_LEFT);
    m_rightMotor = new Servo(HopperConstants.MOTOR_PWM_ID_RIGHT);
  }

  /**
   * Sets the hopper servo position in degrees.
   *
   * @param degrees the target angle
   */
  public void setAngle(double degrees) {
    m_leftMotor.setAngle(degrees);
    m_rightMotor.setAngle(180.0 - degrees);
  }

  /**
   * Gets the current angle of the hopper servo in degrees.
   * 
   * @return the current angle of the servo
   */
  public double getAngle() {
    return m_leftMotor.getAngle();
  }

  @Override
  public void periodic() {
  }
}
