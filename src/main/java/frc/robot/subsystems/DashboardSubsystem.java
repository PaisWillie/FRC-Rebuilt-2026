// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MatchConstants;

public class DashboardSubsystem extends SubsystemBase {
  private static final double DASHBOARD_UPDATE_PERIOD_SECONDS = 0.1;

  private Color colorInactive;
  private Color color;
  private double timeLeft;
  private double previousTime;
  private double lastPublishTimestamp;

  /** Creates a new DashboardSubsystem. */
  public DashboardSubsystem() {
    color = Color.BOTH;
    timeLeft = 0;
    previousTime = 0;
    lastPublishTimestamp = Double.NEGATIVE_INFINITY;
  }
  private enum Color {
    RED,
    BLUE,
    BOTH
  }

  public void setInactiveFirst(char c) {
    colorInactive = c == 'R' ? Color.RED : Color.BLUE;
  }

  private String getColorCode(Color color) {
    if (color == Color.RED) {
      return "#FF0000";
    } else if (color == Color.BLUE) {
      return "#0000FF";
    }
    return "#FFFFFF";
  }

  private void updateAllianceShift() {
    double matchTime = DriverStation.getMatchTime();
    previousTime = timeLeft;
    if (DriverStation.isAutonomous()) {
      timeLeft = matchTime;
    } else if (matchTime <= MatchConstants.MATCH_TIME_TELEOP_START
        && matchTime > MatchConstants.MATCH_TIME_TRANSITION_END) {
      color = (colorInactive == Color.RED) ? Color.BLUE : Color.RED;
      timeLeft = matchTime - MatchConstants.MATCH_TIME_TRANSITION_END;
    } else if (matchTime <= MatchConstants.MATCH_TIME_SHIFT_4_END) {
      timeLeft = matchTime;
      color = Color.BOTH;
    } else {
      timeLeft = (matchTime - 30) % 25;
      if (timeLeft > previousTime) {
        color = (color == Color.RED) ? Color.BLUE : Color.RED;
      }
    }
  }

  @Override
  public void periodic() {
    updateAllianceShift();

    double now = Timer.getFPGATimestamp();
    if (now - lastPublishTimestamp < DASHBOARD_UPDATE_PERIOD_SECONDS) {
      return;
    }

    lastPublishTimestamp = now;
    SmartDashboard.putNumber("Shift time", timeLeft);
    SmartDashboard.putString("Alliance Shift Color", getColorCode(color));
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }
}
