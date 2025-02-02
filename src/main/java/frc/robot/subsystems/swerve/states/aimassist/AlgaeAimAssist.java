package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefSide;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class AlgaeAimAssist implements IRotationalAimAssist, ITranslationalAimAssist {

	private final ReefSide target;

	private final Supplier<Pose2d> robotPoseSupplier;

	public AlgaeAimAssist(ReefSide reefSide, Supplier<Pose2d> robotPoseSupplier) {
		target = reefSide;
		this.robotPoseSupplier = robotPoseSupplier;
	}

	@Override
	public ChassisSpeeds handleAimAssist(ChassisSpeeds chassisSpeeds, Swerve swerve) {
		ChassisSpeeds rotationalAimAssistSpeeds = IRotationalAimAssist.super.handleAimAssist(chassisSpeeds, swerve);
		return ITranslationalAimAssist.super.handleAimAssist(rotationalAimAssistSpeeds, swerve);
	}

	@Override
	public Rotation2d getTargetHeading() {
		return Field.getReefSideMiddle(target).getRotation();
	}

	@Override
	public Translation2d getObjectTranslation() {
		return Field.getReefSideMiddle(target).getTranslation();
	}

	@Override
	public Supplier<Pose2d> getRobotPose() {
		return robotPoseSupplier;
	}

	@Override
	public String getName() {
		return "Algae";
	}
}
