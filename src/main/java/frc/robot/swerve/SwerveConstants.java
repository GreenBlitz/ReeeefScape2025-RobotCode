package frc.robot.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public record SwerveConstants(
	String logPath,
	String stateLogPath,
	String velocityLogPath,
	double velocityAt12VoltsMetersPerSecond,
	Rotation2d maxRotationalVelocityPerSecond,
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
			logPath + "State/",
			logPath + "Velocity/",
			velocityAt12VoltsMetersPerSecond,
			maxRotationalVelocityPerSecond,
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(rotationDegreesPIDConstants.kP, rotationDegreesPIDConstants.kI, rotationDegreesPIDConstants.kD),
			new PPHolonomicDriveController(translationMetersPIDConstants, rotationDegreesPIDConstants)
		);

		this.rotationDegreesPIDController.enableContinuousInput(-MathConstants.HALF_CIRCLE.getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());
	}

	static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND = Rotation2d.fromRotations(0.5);

	public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 4;

	static final double DRIVE_NEUTRAL_DEADBAND = 0.05;
	static final Rotation2d ROTATION_NEUTRAL_DEADBAND = Rotation2d.fromRadians(0.05);
	static final Rotation2d CALIBRATION_MODULE_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
	static final Rotation2d CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND = Rotation2d.fromDegrees(3);

}
