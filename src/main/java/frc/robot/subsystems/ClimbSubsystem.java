// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ElevatorSubsystem;
import frc.robot.subsystems.climb.TongueSubsystem;

public class ClimbSubsystem extends SubsystemBase {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final TongueSubsystem m_tongueSubsystem;

    private enum ClimbPosition {
        NONE, L1, L2, L3
    }

    private ClimbPosition m_currentClimbPosition;
    private boolean m_isClimbAttempted;
    private boolean m_isClimbInterrupted;

    public ClimbSubsystem() {
        m_elevatorSubsystem = new ElevatorSubsystem();
        m_tongueSubsystem = new TongueSubsystem();
        m_currentClimbPosition = ClimbPosition.NONE;
        m_isClimbInterrupted = false;
    }

    private void setClimbPosition(ClimbPosition newPosition) {
        m_currentClimbPosition = newPosition;
    }

    // TODO
    public Command climbToL1() {

        if (m_isClimbInterrupted)
            return Commands.none(); // Prevent climbing if the climb was previously interrupted

        return new InstantCommand(() -> {
            m_isClimbAttempted = true;
        }).andThen(Commands.sequence(Commands.none()) // TODO
                .withName("CLMB - Climb to L1 from ground")
                .onlyIf(() -> m_currentClimbPosition == ClimbPosition.NONE)
                .andThen(Commands.runOnce(() -> setClimbPosition(ClimbPosition.L1))));
    }

    // TODO
    public Command climbToL2() {

        if (m_isClimbInterrupted)
            return Commands.none(); // Prevent climbing if the climb was previously interrupted

        Command L1toL2 = Commands.sequence(Commands.none()) // TODO
                .withName("CLMB - Climb to L2 from L1")
                .andThen(Commands.runOnce(() -> setClimbPosition(ClimbPosition.L2)));

        switch (m_currentClimbPosition) {
            case NONE:
                return Commands.sequence(climbToL1(), L1toL2);
            case L1:
                return L1toL2;
            default:
                return Commands.none();
        }
    }

    public Command climbToL3() {

        if (m_isClimbInterrupted)
            return Commands.none(); // Prevent climbing if the climb was previously interrupted

        Command L2toL3 = Commands.sequence(Commands.none()) // TODO
                .withName("CLMB - Climb to L3 from L2")
                .andThen(Commands.runOnce(() -> setClimbPosition(ClimbPosition.L3)));

        switch (m_currentClimbPosition) {
            case NONE:
                return Commands.sequence(climbToL1(), climbToL2(), L2toL3);
            case L1:
                return Commands.sequence(climbToL2(), L2toL3);
            case L2:
                return L2toL3;
            default:
                return Commands.none();
        }
    }

    public Command interruptClimb() {
        if (!m_isClimbAttempted) {
            return Commands.none(); // Don't interrupt if climb hasn't been attempted
        }

        return new InstantCommand(() -> {
            m_isClimbInterrupted = true;
            m_elevatorSubsystem.stop();
            m_tongueSubsystem.stop();
        }).withName("CLMB - Interrupt Climb");
    }

    @Override
    public void periodic() {

    }
}
