package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    public static final double FIELD_WIDTH = 8.069;
    public static final double FIELD_LENGTH = 16.541;

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.0218614 + Units.inchesToMeters(47.0) / 2.0,
            FIELD_WIDTH / 2);

    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9087646, 4.0345);

    public static final double BLUE_STARTING_LINE_X = 4.0218614;
    public static final double RED_STARTING_LINE_X = 12.519177399999998;
}
