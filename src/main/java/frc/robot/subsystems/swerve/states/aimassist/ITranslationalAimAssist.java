package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public interface ITranslationalAimAssist extends IAimAssist {

	default ChassisSpeeds handleAimAssist(ChassisSpeeds chassisSpeeds, Swerve swerve) {
		return AimAssistMath.getObjectAssistedSpeeds(
			chassisSpeeds,
			getRobotPose().get(),
			getTargetHeading(),
			getObjectTranslation(),
			swerve.getConstants()
		);
	}

	Rotation2d getTargetHeading();

	Translation2d getObjectTranslation();

	Supplier<Pose2d> getRobotPose();

}
