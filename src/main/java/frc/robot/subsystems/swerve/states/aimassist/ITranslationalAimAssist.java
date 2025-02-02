package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.states.DriveRelative;
import frc.robot.subsystems.swerve.states.SwerveState;

import java.util.function.Supplier;

public interface ITranslationalAimAssist extends IAimAssist{

    default Supplier<ChassisSpeeds> handleAimAssist(ChassisSpeeds chassisSpeeds, SwerveConstants swerveConstants, SwerveState swerveState) {
        return () -> AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, getRobotPose().get(), getTargetHeading(), getObjectTranslation(), swerveConstants, swerveState);
    }

    Rotation2d getTargetHeading();

    Translation2d getObjectTranslation();

}
