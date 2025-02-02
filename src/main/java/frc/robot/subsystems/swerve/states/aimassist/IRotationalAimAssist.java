package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Supplier;

public interface IRotationalAimAssist extends IAimAssist {

	default ChassisSpeeds handleAimAssist(ChassisSpeeds chassisSpeeds, Swerve swerve) {
		return AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, swerve.getAllianceRelativeHeading(), getTargetHeading(), swerve.getConstants());
	}

	Rotation2d getTargetHeading();

}
