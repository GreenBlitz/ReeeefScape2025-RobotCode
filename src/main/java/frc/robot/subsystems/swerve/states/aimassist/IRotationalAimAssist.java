package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;


public interface IRotationalAimAssist extends IAimAssist {

	default ChassisSpeeds handleAimAssist(ChassisSpeeds allianceRelativeChassisSpeeds, Swerve swerve) {
		return AimAssistMath
			.getRotationAssistedSpeeds(allianceRelativeChassisSpeeds, swerve.getAllianceRelativeHeading(), getTargetHeading(), swerve.getConstants());
	}

	Rotation2d getTargetHeading();

}
