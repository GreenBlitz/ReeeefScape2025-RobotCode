package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;

public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

    Pose2d getEstimatedPose();

    void restPose(Pose2d newPose);

}
