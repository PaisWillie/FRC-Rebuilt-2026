package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;

public class LimelightWrapper extends Limelight {

    private final boolean isLL4;
    private final PoseEstimate poseEstimateMt2;
    private final PoseEstimate poseEstimateMt1;

    public LimelightWrapper(String limelightName, boolean isLL4) {
        super(limelightName);

        this.isLL4 = isLL4;
        poseEstimateMt2 = new PoseEstimate(this, "botpose_orb_wpiblue", true);
        poseEstimateMt1 = new PoseEstimate(this, "botpose_wpiblue", false);
    }

    /**
     * Retrieve estimated standard deviations for a Megatag 1 estimate
     * 
     * @param poseEstimate the pose estimate from the limelight
     * @return the estimated standard deviations
     */
    private Matrix<N3, N1> getEstimationStdDevsLimelightMT1(
            limelight.networktables.PoseEstimate poseEstimate,
            boolean isLL4) {
        var estStdDevs = VisionConstants.MT1_STDDEV;
        double stddevScalar = 1;

        // Calculate the number of tags, average distance and average ambiguity
        int numTags = 0;
        double avgDist = 0;
        double avgAmbiguity = 0;
        for (var value : poseEstimate.rawFiducials) { // Loop through all the tags detected
            numTags++;
            avgDist += value.distToCamera;
            avgAmbiguity += value.ambiguity;
        }

        // if no tags detected, ignorse the pose by returning very high std devs
        if (numTags == 0) {
            return VecBuilder.fill(1e6, 1e6, 1e6);
        }

        // Calculate the averages
        avgDist /= numTags;
        avgAmbiguity /= numTags;

        // Decrease std devs if limelight is LL4
        if (isLL4) {
            stddevScalar *= .1;
        }

        // If the average ambiguity is too high, return very high std devs to ignore the
        // pose
        if (avgAmbiguity > 0.3) {
            return VecBuilder.fill(1e6, 1e6, 1e6);
        }

        // Scale the standard deviations based on the average ambiguity
        stddevScalar *= (1 + (avgAmbiguity * 5));

        // If the average distance is too far, return very high std devs to ignore the
        // pose
        if (numTags == 1 && avgDist > 2) {
            estStdDevs = VecBuilder.fill(1e6, 1e6, 1e6);
        } else { // Scale the standard deviations based on the average distance
            stddevScalar *= (1 + (avgDist * avgDist / 30));
        }

        // apply the calculated scalar to the standard deviations
        estStdDevs = estStdDevs.times(stddevScalar);

        return estStdDevs;
    }

    /**
     * Retrieve estimated standard deviations for a Megatag 2 estimate
     * 
     * @param poseEstimate the pose estimate from the limelight
     * @return the estimated standard deviations
     */
    private Matrix<N3, N1> getEstimationStdDevsLimelightMT2(
            limelight.networktables.PoseEstimate poseEstimate,
            boolean isLL4) {
        var estStdDevs = VisionConstants.MT2_STDDEV;
        double stddevScalar = 1;

        int numTags = 0;
        double avgDist = 0;
        for (var value : poseEstimate.rawFiducials) {
            numTags++;
            avgDist += value.distToCamera;
        }

        // if no tags detected, ignorse the pose by returning very high std devs
        if (numTags == 0) {
            return VecBuilder.fill(1e6, 1e6, 1e6);
        }

        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            stddevScalar *= (0.65);
        }

        // Decrease std devs if limelight is LL4
        if (isLL4) {
            stddevScalar *= (.8);
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 5) {
            estStdDevs = VecBuilder.fill(1e6, 1e6, 1e6);
        } else {
            stddevScalar *= (1 + (avgDist * avgDist * .2));
        }

        // apply the calculated scalar to the standard deviations
        estStdDevs = estStdDevs.times(stddevScalar);

        return estStdDevs;
    }

    /**
     * Updates the robot's pose estimation using data from the limelight. It
     * retrieves the latest pose estimates from the limelight, checks the number of
     * visible tags and the robot's rotation speed, and if the conditions are
     * favorable, it adds the vision measurements to the swerve drive's pose
     * estimator with appropriate standard deviations.
     * 
     * @param swerveDrive
     * @return true if the pose was updated with vision data, false otherwise
     */
    public boolean updateLocalization(SwerveDrive swerveDrive) {
        getSettings()
                .withRobotOrientation(new Orientation3d(swerveDrive.getGyroRotation3d(),
                        new AngularVelocity3d(RadiansPerSecond.of(swerveDrive.getRobotVelocity().omegaRadiansPerSecond),
                                RadiansPerSecond.of(0),
                                RadiansPerSecond.of(0))))
                .save();

        Optional<PoseEstimate> visionEstimateMt2 = Optional.of(poseEstimateMt2).get().getPoseEstimate();

        boolean updated = false;

        if (visionEstimateMt2.isPresent()) {
            limelight.networktables.PoseEstimate poseEstimate = visionEstimateMt2.get();
            // If we see >0 tags and robot rotates <2 rotations per second
            if (poseEstimate.tagCount > 0
                    && Math.abs(Units.radiansToRotations(swerveDrive.getRobotVelocity().omegaRadiansPerSecond)) < 2) {
                // Add it to the pose estimator.
                swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds,
                        getEstimationStdDevsLimelightMT2(poseEstimate, isLL4));
                updated = true;
            }
        }

        return updated;

        // Only if is limelight 4, add MT1 yaw measurement to the estimator
        // if (isLL4) {
        // Optional<PoseEstimate> visionEstimateMt1 =
        // Optional.of(poseEstimateMt1).get().getPoseEstimate();

        // // If the pose is present
        // visionEstimateMt1.ifPresent((limelight.networktables.PoseEstimate
        // poseEstimate) -> {
        // // And we see >1 tags and robot rotates <2 rotations per second
        // if (poseEstimate.tagCount > 1
        // && Math.abs(
        // Units.radiansToRotations(swerveDrive.getRobotVelocity().omegaRadiansPerSecond))
        // < 2) {
        // // Add it to the pose estimator.
        // swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(),
        // poseEstimate.timestampSeconds,
        // getEstimationStdDevsLimelightMT1(poseEstimate, isLL4));
        // }
        // });
        // }
    }
}