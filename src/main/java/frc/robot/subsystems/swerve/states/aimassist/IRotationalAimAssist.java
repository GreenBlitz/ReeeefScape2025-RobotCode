package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Supplier;

public interface IRotationalAimAssist extends IAimAssist {

    default Supplier<ChassisSpeeds> handleAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, SwerveConstants swerveConstants) {
        return () -> AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotHeading, getTargetHeading(), swerveConstants);
    }

    Rotation2d getTargetHeading();

}
