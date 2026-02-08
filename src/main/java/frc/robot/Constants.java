package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5);
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class IndexerConstants {
        public static final int MOTOR_ID = -1;

        public static final int MOTOR_STATOR_CURRENT_LIMIT = -1;
        public static final int MOTOR_SUPPLY_CURRENT_LIMIT = -1;
    }
}
