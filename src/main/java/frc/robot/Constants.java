package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;

import static edu.wpi.first.units.Units.*;

public final class Constants {

    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5);
    }

    public static final class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class IndexerConstants {
        public static final int MOTOR_CAN_ID = -1;

        public static final int MOTOR_STATOR_CURRENT_LIMIT = -1;
        public static final int MOTOR_SUPPLY_CURRENT_LIMIT = -1;
    }

    public static class IntakeConstants {
        public static final int MOTOR_PWM_ID = -1;
    }

    public static final class FlywheelConstants {
        public static final int MOTOR_ID = -1; // TODO

        public static final Distance DIAMETER_INCHES = Inches.of(4.0); // TODO
        public static final Mass MASS_POUNDS = Pounds.of(1); // TODO

        public static final GearBox GEARBOX = GearBox.fromReductionStages(3, 4); // TODO

        public static final Current STATOR_CURRENT_LIMIT_AMPS = Amps.of(40); // TODO

        public static final double PID_kP = 0.00016541; // TODO
        public static final double PID_kI = 0.0; // TODO
        public static final double PID_kD = 0.0; // TODO

        public static final Time CLOSED_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO
        public static final Time OPEN_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO

        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.27937, 0.089836,
                0.014557); // TODO
        public static final SimpleMotorFeedforward SIM_FEEDFORWARD = new SimpleMotorFeedforward(0.27937, 0.089836,
                0.014557); // TODO

        public static final double SOFT_LIMIT_RPM = 5000; // TODO

        public static final AngularVelocity MAX_VELOCITY_RPM = RPM.of(5000); // TODO
        public static final AngularVelocity SIM_MAX_VELOCITY_RPM = RPM.of(7500); // TODO

        public static final AngularAcceleration MAX_ACCELERATION_RPS2 = RotationsPerSecondPerSecond.of(2500); // TODO
    }
}
