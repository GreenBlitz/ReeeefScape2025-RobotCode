package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;

public record SwerveConstants(
	String logPath,
	String stateLogPath,
	String velocityLogPath,
	ChassisSpeeds maxSpeeds,
	PIDController xMetersPIDController,
	PIDController yMetersPIDController,
	PIDController rotationDegreesPIDController,
	PPHolonomicDriveController pathPlannerHolonomicDriveController
) {

	public SwerveConstants(
		String logPath,
		double velocityAt12VoltsMetersPerSecond,
		Rotation2d maxRotationalVelocityPerSecond,
		PIDConstants translationMetersPIDConstants,
		PIDConstants rotationDegreesPIDConstants
	) {
		this(
			logPath,
			logPath + "/State",
			logPath + "/Velocity",
			new ChassisSpeeds(velocityAt12VoltsMetersPerSecond, velocityAt12VoltsMetersPerSecond, maxRotationalVelocityPerSecond.getRadians()),
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(rotationDegreesPIDConstants.kP, rotationDegreesPIDConstants.kI, rotationDegreesPIDConstants.kD),
			new PPHolonomicDriveController(translationMetersPIDConstants, rotationDegreesPIDConstants)
		);

		this.rotationDegreesPIDController
			.enableContinuousInput(MathConstants.HALF_CIRCLE.unaryMinus().getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());
	}

	static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND = Rotation2d.fromRotations(0.5);

	public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 4;

	static final Pose2d DEADBANDS = new Pose2d(0.05, 0.05, Rotation2d.fromRadians(0.05));
	static final Rotation2d CALIBRATION_MODULE_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
	static final Rotation2d CALIBRATION_MODULE_ANGULAR_VELOCITY_PER_SECOND_DEADBAND = Rotation2d.fromDegrees(3);

}
