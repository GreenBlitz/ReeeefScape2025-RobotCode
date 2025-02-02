package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;

public interface IAimAssist {

	ChassisSpeeds handleAimAssist(ChassisSpeeds chassisSpeeds, Swerve swerve);

	String getName();

}
