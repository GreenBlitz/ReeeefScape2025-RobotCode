package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefSide;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Supplier;

public class ReefAimAssist implements IRotationalAimAssist {

    private final ReefSide target;

    private final Supplier<Pose2d> robotPoseSupplier;

    public ReefAimAssist(ReefSide reefSide, Supplier<Pose2d> robotPoseSupplier) {
        target = reefSide;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public Rotation2d getTargetHeading() {
        return Field.getReefSideMiddle(target).getRotation();
    }

    @Override
    public Supplier<Pose2d> getRobotPose() {
        return robotPoseSupplier;
    }
}
