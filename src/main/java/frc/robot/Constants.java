package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public final class Constants {

    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5); // TODO
    }

    public static final class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
    }

    public static class IndexerConstants {
        public static final int MOTOR_CAN_ID = -1; // TODO

        public static final int MOTOR_STATOR_CURRENT_LIMIT = -1; // TODO
        public static final int MOTOR_SUPPLY_CURRENT_LIMIT = -1; // TODO
    }

    public static class IntakeConstants {
        public static final int ROLLER_MOTOR_PWM_ID = -1; // TODO
        public static final double ROLLER_DEFAULT_SPEED_DUTY_CYCLE = 0.0; // TODO

        public static class LinearConstants {
            public static final int LINEAR_MOTOR_CAN_ID = -1;
            public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
            public static final Distance MOTOR_CIRCUMFERENCE_METERS = Meters.of(Inches.of(0.25).in(Meters) * 22); // TODO
            public static final double PID_kP = 4.0; // TODO
            public static final double PID_kI = 0.0; // TODO
            public static final double PID_kD = 0.0; // TODO
            public static final LinearVelocity MAX_VELOCITY_MPS = MetersPerSecond.of(0.5); // TODO
            public static final LinearAcceleration MAX_ACCELERATION_MPS2 = MetersPerSecondPerSecond.of(0.5); // TODO
            public static final Distance SOFT_LIMIT_MIN_METERS = Meters.of(0); // TODO
            public static final Distance SOFT_LIMIT_MAX_METERS = Meters.of(2); // TODO
            public static final GearBox GEARBOX = GearBox.fromReductionStages(3, 4); // TODO
            public static final Current STATOR_CURRENT_LIMIT_AMPS = Amps.of(40); // TODO
            public static final Time CLOSED_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO
            public static final Time OPEN_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO
            public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0, 0); // TODO
            public static final Distance STARTING_HEIGHT_METERS = Meters.of(0.5); // TODO
            public static final Distance HARD_LIMIT_MIN_METERS = Meters.of(0); // TODO
            public static final Distance HARD_LIMIT_MAX_METERS = Meters.of(3); // TODO
            public static final Mass MECHANISM_MASS_POUNDS = Pounds.of(16); // TODO
            public static final Distance ROBOT_MAX_HEIGHT_METERS = Meters.of(1.5); // TODO
            public static final Distance ROBOT_MAX_LENGTH_METERS = Meters.of(0.75); // TODO
            public static final Translation3d RELATIVE_POSITION_METERS = new Translation3d(Meters.of(-0.25),
                    Meters.of(0),
                    Meters.of(0.5)); // TODO
        }
    }

    public static final class FlywheelConstants {
        public static final int MOTOR_ID = -1; // TODO
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

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

    public static final class HoodConstants {
        public static final int MOTOR_CAN_ID = -1; // TODO
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

        public static final int ENCODER_CAN_ID = -1; // TODO

        public static final GearBox GEARBOX = GearBox.fromReductionStages(3, 4); // TODO

        public static final Current STATOR_CURRENT_LIMIT_AMPS = Amps.of(40); // TODO

        public static final double PID_kP = 0.00016541; // TODO
        public static final double PID_kI = 0.0; // TODO
        public static final double PID_kD = 0.0; // TODO

        public static final AngularVelocity MAX_VELOCITY_RPM = RPM.of(5000); // TODO
        public static final AngularAcceleration MAX_ACCELERATION_RPS2 = RotationsPerSecondPerSecond.of(2500); // TODO

        public static final Time CLOSED_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO
        public static final Time OPEN_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.27937, 0.089836,
                0.014557); // TODO
        public static final SimpleMotorFeedforward SIM_FEEDFORWARD = new SimpleMotorFeedforward(0.27937, 0.089836,
                0.014557); // TODO

        public static final Angle SOFT_LIMIT_MIN = Degrees.of(5); // TODO
        public static final Angle SOFT_LIMIT_MAX = Degrees.of(100); // TODO

        public static final Angle HARD_LIMIT_MIN = Degrees.of(0); // TODO
        public static final Angle HARD_LIMIT_MAX = Degrees.of(120); // TODO

        public static final Voltage SYSID_MAX_VOLTAGE = Volts.of(4.0); // TODO
        public static final Velocity<VoltageUnit> SYSID_STEP = Volts.per(Second).of(0.5); // TODO
        public static final Time SYSID_DURATION = Seconds.of(8.0); // TODO
        public static final boolean EXTERNAL_ENCODER_INVERTED = false; // TODO
        public static final double EXTERNAL_ENCODER_GEARING = 17;
        public static final Angle EXTERNAL_ENCODER_ZERO_OFFSET = Degrees.of(0); // TODO
    }

    public static final class FeederConstants {
        public static final int MOTOR_CAN_ID = -1; // TODO

        public static final int MOTOR_STATOR_CURRENT_LIMIT = -1; // TODO
        public static final int MOTOR_SUPPLY_CURRENT_LIMIT = -1; // TODO
    }

    public static final class HopperConstants {
        public static final int MOTOR_ID = -1; // TODO
    }

    public static final class ClimbConstants {
        public static final int MOTOR_CAN_ID = 2; // TODO
        public static final DCMotor MOTOR = DCMotor.getNEO(1); // TODO
        public static final Distance CHAIN_PITCH = Inches.of(0.25); // TODO
        public static final int TOOTH_COUNT = 22; // TODO
        public static final MechanismGearing GEARBOX = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // TODO
        public static final Mass MASS = Pounds.of(16); // TODO
        public static final Distance STARTING_HEIGHT = Meters.of(0); // TODO
        public static final Distance SOFT_LOWER_LIMIT = Meters.of(0); // TODO
        public static final Distance SOFT_UPPER_LIMIT = Meters.of(2); // TODO
        public static final Distance HARD_LOWER_LIMIT = Meters.of(0); // TODO
        public static final Distance HARD_UPPER_LIMIT = Meters.of(3); // TODO
        public static final double PID_kP = 1; // TODO
        public static final double PID_kI = 0; // TODO
        public static final double PID_kD = 0; // TODO
        public static final double FEEDFORWARD_kS = 0; // TODO
        public static final double FEEDFORWARD_kG = 0; // TODO
        public static final double FEEDFORWARD_kV = 0; // TODO
        public static final double FEEDFORWARD_kA = 0; // TODO
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40); // TODO
        public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25); // TODO
        public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25); // TODO
        public static final Voltage SYSID_MAX_VOLTAGE = Volts.of(12); // TODO
        public static final Velocity<VoltageUnit> SYSID_STEP = Volts.of(12).per(Second); // TODO
        public static final Time SYSID_DURATION = Second.of(30); // TODO
        public static final Time HOMING_DEBOUNCE_TIME = Seconds.of(0.4); // TODO
        public static final Voltage HOMING_RUN_VOLTS = Volts.of(-2); // TODO
        public static final AngularVelocity HOMING_VELOCITY_THRESHOLD = DegreesPerSecond.of(2); // TODO
    }

    // Set the telemetry verbosity for YAMS subsystems
    public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.HIGH; // TODO: Consider setting to
                                                                                          // LOW or MEDIUM for
                                                                                          // competition to reduce
                                                                                          // network traffic
}
