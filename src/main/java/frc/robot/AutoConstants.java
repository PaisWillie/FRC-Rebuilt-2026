package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
    public static final double DEFAULT_WAYPOINT_TOLERANCE = 0.25;

    public enum Position {
        STARTING_LINE_RIGHT,
        NEUTRAL_RIGHT_1,
        NEUTRAL_RIGHT_2,
        NEUTRAL_RIGHT_3,
        ALLIANCE_RIGHT_1,
        NEUTRAL_RIGHT_4,
        NEUTRAL_RIGHT_5,
        NEUTRAL_RIGHT_6
    }

    // Create map position enum to Pose2D
    public static Map<Position, Pose2d> positionToPose = Map.ofEntries(
            Map.entry(Position.STARTING_LINE_RIGHT, new Pose2d(4.0218614, 0.632, new Rotation2d(0))),
            Map.entry(Position.NEUTRAL_RIGHT_1,
                    new Pose2d(8.02169132232666, 0.632, new Rotation2d(-1.3203533077139966))),
            Map.entry(Position.NEUTRAL_RIGHT_2,
                    new Pose2d(7.77288293838501, 3.4187989234924316,
                            new Rotation2d(-1.3203533077139966))),
            Map.entry(Position.NEUTRAL_RIGHT_3,
                    new Pose2d(5.869497776031494, 0.632, new Rotation2d(Degrees.of(90)))),
            Map.entry(Position.ALLIANCE_RIGHT_1,
                    new Pose2d(4.015, 0.632, new Rotation2d(Degrees.of(80.25)))),
            Map.entry(Position.NEUTRAL_RIGHT_4,
                    new Pose2d(6.25515079498291, 0.632, new Rotation2d(Degrees.of(90)))),
            Map.entry(Position.NEUTRAL_RIGHT_5,
                    new Pose2d(5.857057571411133, 1.9135074615478516, new Rotation2d(-1.100613104652783))),
            Map.entry(Position.NEUTRAL_RIGHT_6,
                    new Pose2d(5.857057571411133, 3.4187989234924316, new Rotation2d(-1.100613104652783))));
}
