package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public final class Constants {

    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16); // TODO: Try increasing this to see if it
                                                                       // does
                                                                       // anything

        public static final double AUTO_AIM_VELOCITY_COMPENSATION_FACTOR = 1.2; // TODO
        public static final Angle AUTO_AIM_ANGLE_TARGET_ERROR = Degrees.of(5); // TODO
        public static final double AUTO_AIM_SCALE_TRANSLATION = 0.3; // TODO

        // Extra distance past the starting line (towards the neutral zone) that would
        // still be considered "in the alliance zone" for the purposes of auto-aiming.
        public static final double ALLIANCE_ZONE_TOLERANCE_TO_STARTING_LINE = 0.1281386;

        public static final Translation2d BLUE_LEFT_FEEDING_TARGET = new Translation2d(0.567625 + 0.5, 6.05175);
        public static final Translation2d BLUE_RIGHT_FEEDING_TARGET = new Translation2d(0.567625 + 0.5,
                2.01725);
        public static final Translation2d RED_LEFT_FEEDING_TARGET = new Translation2d(15.973375 - 0.5, 2.01725);
        public static final Translation2d RED_RIGHT_FEEDING_TARGET = new Translation2d(15.973375 - 0.5,
                6.05175);

        public static final Pose2d RED_LEFT_TOWER_CLIMB_POS = new Pose2d(15.105, 3.885,
                new Rotation2d(Degree.of(0)));
        public static final Pose2d RED_RIGHT_TOWER_CLIMB_POS = new Pose2d(15.105, 4.764,
                new Rotation2d(Degree.of(0)));

        public static final Pose2d BLUE_LEFT_TOWER_CLIMB_POS = new Pose2d(1.428, 4.175,
                new Rotation2d(Degree.of(180)));
        public static final Pose2d BLUE_RIGHT_TOWER_CLIMB_POS = new Pose2d(1.428, 3.317,
                new Rotation2d(Degree.of(180)));

        public static final double DRIVE_TO_POSE_TRANSLATION_kP = 2.3;
        public static final double DRIVE_TO_POSE_TRANSLATION_kI = 0;
        public static final double DRIVE_TO_POSE_TRANSLATION_kD = 0;
        public static final double DRIVE_TO_POSE_TRANSLATION_MAX_VELOCITY = 0.5;
        public static final double DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION = 0.1;

        public static final double DRIVE_TO_POSE_ROTATION_kP = 2.7;
        public static final double DRIVE_TO_POSE_ROTATION_kI = 0;
        public static final double DRIVE_TO_POSE_ROTATION_kD = 0;
        public static final double DRIVE_TO_POSE_ROTATION_MAX_VELOCITY_RAD = Units.degreesToRadians(360);
        public static final double DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD = Units.degreesToRadians(180);
    }

    public static final class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
    }

    public static class IndexerConstants {
        public static final int MOTOR_CAN_ID = 19;

        public static final int MOTOR_STATOR_CURRENT_LIMIT = 40;
        public static final int MOTOR_SUPPLY_CURRENT_LIMIT = 40;

        public static final double INDEXER_FULL_SPEED = 1; // TODO
        public static final double INDEXER_HALF_SPEED = 0; // TODO
    }

    public static class IntakeConstants {
        public static class IntakeRollerConstants {
            public static final int MOTOR_PWM_ID = 1; // TODO

            public static final double SPEED = 0.5; // TODO
        }

        public static class LinearIntakeConstants {
            public static final int MOTOR_CAN_ID = 18;
            public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

            public static final int LEFT_EXTENDED_LIMIT_SWITCH_DIO = 3;
            public static final int LEFT_RETRACTED_LIMIT_SWITCH_DIO = 4;
            public static final int RIGHT_EXTENDED_LIMIT_SWITCH_DIO = 5;
            public static final int RIGHT_RETRACTED_LIMIT_SWITCH_DIO = 7;

            public static final boolean INVERT_MOTOR = true;
            public static final MotorMode IDLE_MODE = MotorMode.COAST;

            public static final Distance MOTOR_CIRCUMFERENCE = Inches.of(1).times(Math.PI);
            public static final GearBox GEARBOX = GearBox.fromStages("54:16", "18:12");
            public static final Mass MECHANISM_MASS = Pounds.of(6.825);
            public static final Angle MECHANISM_ANGLE = Degrees.of(-24.159);

            public static final double PID_kP = 5.5;
            public static final double PID_kI = 0.0;
            public static final double PID_kD = 0.0;

            public static final double SIM_PID_kP = 10.0; // TODO
            public static final double SIM_PID_kI = 0.0; // TODO
            public static final double SIM_PID_kD = 0.0; // TODO

            public static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0.32176, -0.04,
                    0.6148, 0.021608); // TODO: Tune this via SysID

            public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(2); // TODO
            public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(6.5); // TODO

            public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
            public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);

            public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(10);

            public static final Distance SOFT_LIMIT_MIN = Meters.of(0);
            public static final Distance SOFT_LIMIT_MAX = Meters.of(0.31);
            public static final Distance HARD_LIMIT_MIN = Meters.of(0);
            public static final Distance HARD_LIMIT_MAX = Meters.of(0.31);

            public static final Distance EXTENDED_POSITION = SOFT_LIMIT_MAX;
            public static final Distance MIDPOINT_POSITION = Meters.of(0.15);
            public static final Distance POSITION_TARGET_ERROR = Inches.of(0.5);
            public static final Distance RETRACTED_POSITION = SOFT_LIMIT_MIN;

            public static final Distance SHUFFLE_CLOSE_POSITION = Meters.of(0.025);
            public static final Distance SHUFFLE_FAR_POSITION = MIDPOINT_POSITION.plus(Meters.of(0.05));
            public static final Distance SHUFFLE_FURTHEST_POSITION = EXTENDED_POSITION
                    .minus(Meters.of(0.05));

            public static final Translation3d RELATIVE_POSITION = new Translation3d(Inches.of(0),
                    Inches.of(10.682),
                    Inches.of(22.767 / 2));
        }
    }

    public static final class ShooterConstants {
        public static enum ShooterZone {
            ZONE_1,
            ZONE_2,
            ZONE_3,
            ZONE_4
        }

        public static final Map<ShooterZone, Map<Distance, Angle>> SHOOTER_DISTANCE_TO_HOOD_ANGLE = Map
                .ofEntries(
                        // 4000 RPM
                        Map.entry(ShooterZone.ZONE_1, Map.ofEntries(
                                Map.entry(Meter.of(1.6698), Degrees.of(14.0 - 5.5)),
                                Map.entry(Meter.of(1.9717), Degrees.of(17.5 - 5.5)),
                                Map.entry(Meter.of(2.19499), Degrees.of(20.0 - 5.5)),
                                Map.entry(Meter.of(2.4258), Degrees.of(22.5 - 5.5)),
                                Map.entry(Meter.of(2.62955), Degrees.of(24.0 - 5.5)),
                                Map.entry(Meter.of(2.802), Degrees.of(27.5 - 5.5)),
                                Map.entry(Meter.of(2.9945), Degrees.of(30.0 - 5.5)),
                                Map.entry(Meter.of(3.531), Degrees.of(36.0 - 5.5)),
                                Map.entry(Meter.of(3.2396), Degrees.of(33.0 - 5.5))

                        )),
                        // 4775 RPM
                        Map.entry(ShooterZone.ZONE_2, Map.ofEntries(
                                Map.entry(Meter.of(3.6212), Degrees.of(24.5 - 5.5)),
                                Map.entry(Meter.of(4.0396), Degrees.of(27.2 - 5.5)),
                                Map.entry(Meter.of(4.3405), Degrees.of(30.0 - 5.5)),
                                Map.entry(Meter.of(4.660), Degrees.of(34.0 - 5.5)))),

                        Map.entry(ShooterZone.ZONE_3, Map.ofEntries(
                                Map.entry(Meter.of(7.5783), Degrees.of(25)),
                                Map.entry(Meter.of(8.658), Degrees.of(35)),
                                Map.entry(Meter.of(9.9129), Degrees.of(42)),
                                Map.entry(Meter.of(10.1536), Degrees.of(42)))),

                        // Make everything >11m at max angle
                        // TODO: Could probably change this to one entry (requires testing)
                        Map.entry(ShooterZone.ZONE_4, Map.ofEntries(
                                Map.entry(Meter.of(11), HoodConstants.SOFT_LIMIT_MAX),
                                Map.entry(Meter.of(12), HoodConstants.SOFT_LIMIT_MAX),
                                Map.entry(Meter.of(13), HoodConstants.SOFT_LIMIT_MAX),
                                Map.entry(Meter.of(14),
                                        HoodConstants.SOFT_LIMIT_MAX))));

        public static final Map<ShooterZone, AngularVelocity> SHOOTER_MIN_DISTANCE_TO_FLYWHEEL_RPM = Map
                .ofEntries(
                        Map.entry(ShooterZone.ZONE_1, RPM.of(4000)),
                        Map.entry(ShooterZone.ZONE_2, RPM.of(4775)),
                        Map.entry(ShooterZone.ZONE_3, RPM.of(6700)),
                        Map.entry(ShooterZone.ZONE_4, RPM.of(7650)));

        public static final Map<Distance, ShooterZone> MIN_DISTANCE_TO_FLYWHEEL_SPEED_ZONE = Map
                .ofEntries(
                        Map.entry(Meters.of(0), ShooterZone.ZONE_1),
                        Map.entry(Meters.of(3.6212), ShooterZone.ZONE_2),
                        Map.entry(Meters.of(8.2705), ShooterZone.ZONE_3),
                        Map.entry(Meters.of(12.40575), ShooterZone.ZONE_4));

        public static final Map<ShooterZone, InterpolatingDoubleTreeMap> SHOOTER_DISTANCE_TO_HOOD_ANGLE_INTERPOLATION = Map
                .ofEntries(
                        Map.entry(ShooterZone.ZONE_1,
                                createHoodInterpolationMap(ShooterZone.ZONE_1)),
                        Map.entry(ShooterZone.ZONE_2,
                                createHoodInterpolationMap(ShooterZone.ZONE_2)),
                        Map.entry(ShooterZone.ZONE_3,
                                createHoodInterpolationMap(ShooterZone.ZONE_3)),
                        Map.entry(ShooterZone.ZONE_4,
                                createHoodInterpolationMap(ShooterZone.ZONE_4)));

        private static InterpolatingDoubleTreeMap createHoodInterpolationMap(ShooterZone zone) {
            InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
            SHOOTER_DISTANCE_TO_HOOD_ANGLE.get(zone).forEach(
                    (distance, angle) -> map.put(distance.in(Meters), angle.in(Degrees)));
            return map;
        }

        public static final class FlywheelConstants {
            public static final int LEADER_MOTOR_CAN_ID = 20;
            public static final int FOLLOWER_MOTOR_CAN_ID = 21;

            public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(2);
            public static final GearBox GEARBOX = GearBox.fromStages("18:24");

            public static final MotorMode IDLE_MODE = MotorMode.COAST;

            public static final boolean LEADER_MOTOR_INVERTED = false; // TODO
            public static final boolean FOLLOWER_MOTOR_INVERTED = true;

            public static final Distance DIAMETER_INCHES = Inches.of(3);
            public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0006438072);

            public static final Current STATOR_CURRENT_LIMIT_AMPS = Amps.of(80);
            public static final Current SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(40);

            public static final double PID_kP = 0.01; // SysID gave 3386E-05
            public static final double PID_kI = 0.0; // TODO
            public static final double PID_kD = 0.0; // TODO

            public static final double SIM_PID_kP = 0.001; // TODO
            public static final double SIM_PID_kI = 0.0; // TODO
            public static final double SIM_PID_kD = 0.0; // TODO

            public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.23709,
                    0.090625,
                    0.0090964);
            public static final SimpleMotorFeedforward SIM_FEEDFORWARD = new SimpleMotorFeedforward(0.24,
                    0.09, 0.007553); // TODO

            public static final Time CLOSED_LOOP_RAMP_RATE_SEC = Seconds.of(0.25);
            public static final Time OPEN_LOOP_RAMP_RATE_SEC = Seconds.of(0.25);

            public static final AngularVelocity MAX_VELOCITY_RPM = RPM.of(8000);
            public static final AngularVelocity SIM_MAX_VELOCITY_RPM = RPM.of(8000);
            public static final AngularAcceleration MAX_ACCELERATION_RPS2 = RotationsPerSecondPerSecond
                    .of(173);

            public static final AngularVelocity SOFT_LIMIT_RPM = RPM.of(8000); // TODO

            public static final AngularVelocity DEFAULT_VELOCITY = RPM.of(4000);;

            public static final AngularVelocity RPM_TARGET_ERROR = RPM.of(100);
            public static final AngularVelocity RPM_TARGET_ERROR_WHILE_FEEDING = RPM.of(250);

            public static final Time AT_RPM_DEBOUNCE_TIME = Seconds.of(0.5); // TODO

            public static final Translation3d RELATIVE_POSITION = new Translation3d(Inches.of(-5.087),
                    Inches.of(0),
                    Inches.of(17.912 / 2));
        }

        public static final class HoodConstants {
            public static final int MOTOR_CAN_ID = 22;
            public static final int ENCODER_CAN_ID = 6;

            public static final DCMotor MOTOR = DCMotor.getKrakenX44Foc(1);
            public static final GearBox GEARBOX = GearBox.fromReductionStages(80.0 / 14.0, 24.0 / 18.0,
                    170.0 / 10.0);

            public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

            public static final double PID_kP = 35.673;
            public static final double PID_kI = 0.0;
            public static final double PID_kD = 5.251;

            public static final double SIM_PID_kP = 16; // TODO
            public static final double SIM_PID_kI = 7; // TODO
            public static final double SIM_PID_kD = 1; // TODO

            public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(
                    0.25399,
                    11.909,
                    0.44218);
            public static final SimpleMotorFeedforward SIM_FEEDFORWARD = new SimpleMotorFeedforward(
                    0,
                    0,
                    0); // TODO

            public static final Time CLOSED_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO
            public static final Time OPEN_LOOP_RAMP_RATE_SEC = Seconds.of(0.25); // TODO

            public static final AngularVelocity MAX_VELOCITY_RPM = RPM.of(6000); // TODO
            public static final AngularAcceleration MAX_ACCELERATION_RPS2 = RotationsPerSecondPerSecond
                    .of(0.314); // TODO

            public static final Angle SOFT_LIMIT_MIN = Degrees.of(0);
            public static final Angle SOFT_LIMIT_MAX = Degrees.of(42);
            public static final Angle HARD_LIMIT_MIN = Degrees.of(0);
            public static final Angle HARD_LIMIT_MAX = Degrees.of(42.1875);

            public static final Angle DEFAULT_ANGLE = Degrees.of(8);
            public static final Angle LOWER_HOOD_ANGLE = Degrees.of(15);

            public static final Distance LENGTH = Inches.of(8.5);
            public static final Mass MASS = Pounds.of(4.39);

            public static final boolean EXTERNAL_ENCODER_INVERTED = true; // TODO
            public static final double EXTERNAL_ENCODER_GEARING = 17;
            public static final Angle EXTERNAL_ENCODER_ZERO_OFFSET = Degrees.of(163.8); // TODO

            public static final Angle ANGLE_TARGET_ERROR = Degrees.of(2.5);
            public static final Angle ANGLE_TARGET_ERROR_WHILE_FEEDING = Degrees.of(3.5);
            public static final double AT_ANGLE_DEBOUNCE_TIME = 0.2; // TODO

            public static final Translation3d RELATIVE_POSITION = new Translation3d(Inches.of(-5.087),
                    Inches.of(9.017),
                    Inches.of(17.912 / 2));
        }

        public static final class FeederConstants {
            public static final int MOTOR_CAN_ID = 23;
            public static final int BEAM_BREAK_DIO_PORT = 2; // TODO

            public static final AngularVelocity FEEDER_SPEED = RPM.of(725);
            public static final AngularVelocity REVERSE_SPEED = RPM.of(-725);

            public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
            public static final GearBox GEARBOX = GearBox.fromStages("5:1", "24:15");

            public static final MotorMode IDLE_MODE = MotorMode.BRAKE;

            public static final boolean MOTOR_INVERTED = true;

            public static final Distance DIAMETER_INCHES = Inches.of(4);
            public static final MomentOfInertia MOI = KilogramSquareMeters.of(
                    0.0009446408);

            public static final Current STATOR_CURRENT_LIMIT_AMPS = Amps.of(60);
            public static final Current SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(50);

            public static final double PID_kP = 0.0; // TODO
            public static final double PID_kI = 0.0; // TODO
            public static final double PID_kD = 0.0; // TODO

            public static final double SIM_PID_kP = 0.075; // TODO
            public static final double SIM_PID_kI = 0.0; // TODO
            public static final double SIM_PID_kD = 0.0; // TODO

            public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.1517,
                    0.96378,
                    0.017578);
            public static final SimpleMotorFeedforward SIM_FEEDFORWARD = new SimpleMotorFeedforward(2.25,
                    0.0, 0.0); // TODO

            public static final Time CLOSED_LOOP_RAMP_RATE_SEC = Seconds.of(0.25);
            public static final Time OPEN_LOOP_RAMP_RATE_SEC = Seconds.of(0.25);

            public static final AngularVelocity MAX_VELOCITY_RPM = RPM.of(725);
            public static final AngularVelocity SIM_MAX_VELOCITY_RPM = RPM.of(725);
            public static final AngularAcceleration MAX_ACCELERATION_RPS2 = RotationsPerSecondPerSecond
                    .of(99999); // TODO

            public static final AngularVelocity SOFT_LIMIT_RPM = RPM.of(4338);
        }
    }

    public static final class HopperConstants {
        public static final int MOTOR_PWM_ID_LEFT = 2; // TODO
        public static final int MOTOR_PWM_ID_RIGHT = 3; // TODO

        public static final Angle EXPANDED_ANGLE = Degrees.of(90); // TODO
        public static final Angle RETRACT_ANGLE = Degrees.of(0); // TODO
    }

    public static final class ClimbConstants {
        public static final class ElevatorConstants {
            public static final int LEADER_MOTOR_CAN_ID = 55;
            public static final int FOLLOWER_MOTOR_CAN_ID = 54;

            public static final DCMotor LEADER_MOTOR = DCMotor.getFalcon500Foc(1);
            public static final DCMotor FOLLOWER_MOTOR = DCMotor.getFalcon500Foc(1);

            public static final MechanismGearing GEARBOX = new MechanismGearing(
                    GearBox.fromReductionStages(12));

            public static final Distance CHAIN_PITCH = Inches.of(0.25);
            public static final int TOOTH_COUNT = 12;
            public static final Mass MASS = Pounds.of(5); // TODO

            public static final double PID_kP = 5;
            public static final double PID_kI = 0;
            public static final double PID_kD = 0;

            public static final double SIM_PID_kP = 1; // TODO
            public static final double SIM_PID_kI = 0; // TODO
            public static final double SIM_PID_kD = 0; // TODO

            public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(19.34);
            public static final LinearAcceleration MAX_ACCELERATION = InchesPerSecondPerSecond.of(174.65);

            public static final double FEEDFORWARD_kS = 0.24248;
            public static final double FEEDFORWARD_kG = 0.0059565;
            public static final double FEEDFORWARD_kV = 1.4656;
            public static final double FEEDFORWARD_kA = 0.022498;

            public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);

            public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25); // TODO
            public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25); // TODO

            public static final Distance SOFT_LOWER_LIMIT = Meters.of(0);
            public static final Distance SOFT_UPPER_LIMIT = Meters.of(0.42);
            public static final Distance HARD_LOWER_LIMIT = Meters.of(0);
            public static final Distance HARD_UPPER_LIMIT = Meters.of(0.420439);

            public static final Distance STARTING_HEIGHT = SOFT_LOWER_LIMIT;
            public static final Distance FIRST_LEVEL_HEIGHT = Meters.of(0.20); // TODO
            public static final Distance SECOND_LEVEL_HEIGHT = Meters.of(0.30); // TODO

            public static final Time HOMING_DEBOUNCE_TIME = Seconds.of(0.4); // TODO
            public static final Voltage HOMING_RUN_VOLTS = Volts.of(-2); // TODO
            public static final AngularVelocity HOMING_VELOCITY_THRESHOLD = DegreesPerSecond.of(2); // TODO
        }

        public static final class TongueConstants {
            public static final int MOTOR_CAN_ID = 9; // TODO

            public static final Distance MECHANISM_CIRCUMFERENCE = Meters
                    .of(Inches.of(0.25).in(Meters) * 22); // TODO
            public static final GearBox GEARBOX = GearBox.fromReductionStages(3, 4); // TODO
            public static final Mass MECHANISM_MASS = Pounds.of(16); // TODO

            public static final double PID_kP = 4.0; // TODO
            public static final double PID_kI = 0.0; // TODO
            public static final double PID_kD = 0.0; // TODO

            public static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0, 0, 0); // TODO

            public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(0.5); // TODO
            public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.5); // TODO

            public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25); // TODO

            public static final Current STATOR_CURRENT_LIMIT = Amps.of(40); // TODO

            public static final Distance STARTING_HEIGHT = Meters.of(0.5); // TODO
            public static final Distance SOFT_LIMIT_MIN = Meters.of(0); // TODO
            public static final Distance SOFT_LIMIT_MAX = Meters.of(2); // TODO
            public static final Distance HARD_LIMIT_MIN = Meters.of(0); // TODO
            public static final Distance HARD_LIMIT_MAX = Meters.of(3); // TODO

            public static final Translation3d RELATIVE_POSITION = new Translation3d(Meters.of(-0.25),
                    Meters.of(0),
                    Meters.of(0.5)); // TODO
        }
    }

    public static final class MechanismPositionConstants {
        public static final Distance ROBOT_MAX_HEIGHT = Inches.of(21.729);
        public static final Distance ROBOT_MAX_LENGTH = Inches.of(27);
    }

    public static final class VisionConstants {
        // MT1 is configured to be effectively ignored for X/Y position (very large
        // stddev) while still being trusted for rotation. The 1e6 X/Y values indicate
        // extremely high uncertainty in translation so pose estimators will down‑weight
        // MT1's position contribution, but the relatively small rotational stddev (~3
        // degrees) allows MT1 to meaningfully contribute to heading estimation.
        public static final Matrix<N3, N1> MT1_STDDEV = VecBuilder.fill(1e6, 1e6, Math.PI / 60);
        // MT2 is the complementary measurement source: it is trusted for X/Y
        // translation (small stddevs) and effectively ignored for rotation (very large
        // stddev). Together, these settings implement "use only x/y from MT2" and "use
        // only rotation from MT1" when fusing measurements.
        public static final Matrix<N3, N1> MT2_STDDEV = VecBuilder.fill(0.5, 0.5, 1e6);
    }

    public static final class MatchConstants {
        public static final double TRANSITION_SHIFT = 10;
        public static final double SHIFT_1 = 25;
        public static final double SHIFT_2 = 25;
        public static final double SHIFT_3 = 25;
        public static final double SHIFT_4 = 25;
        public static final double END_GAME = 30;

        public static final double MATCH_TIME_TELEOP_START = 140;
        public static final double MATCH_TIME_TRANSITION_END = 130;
        public static final double MATCH_TIME_SHIFT_1_END = 105;
        public static final double MATCH_TIME_SHIFT_2_END = 80;
        public static final double MATCH_TIME_SHIFT_3_END = 55;
        public static final double MATCH_TIME_SHIFT_4_END = 30;
        public static final double MATCH_TIME_END_GAME_END = 0;
    }

    // Consider setting to LOW or MEDIUM for competition to reduce network traffic
    // Set the telemetry verbosity for YAMS subsystems
    public static final SmartMotorControllerConfig.TelemetryVerbosity TELEMETRY_VERBOSITY = SmartMotorControllerConfig.TelemetryVerbosity.LOW;

    public static final SwerveDriveTelemetry.TelemetryVerbosity SWERVE_TELEMETRY_VERBOSITY = SwerveDriveTelemetry.TelemetryVerbosity.POSE;
    public static final boolean TELEMETRY = false; // Set to false for competition to reduce network traffic
}