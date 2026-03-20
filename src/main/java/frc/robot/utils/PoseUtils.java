package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;

public class PoseUtils {
    public static Pose2d flipX(Pose2d pose) {
        return new Pose2d(
                pose.getX(),
                FieldConstants.FIELD_WIDTH - pose.getY(),
                pose.getRotation().times(-1));
    }
}
