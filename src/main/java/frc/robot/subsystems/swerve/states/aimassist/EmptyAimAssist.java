package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;

public class EmptyAimAssist implements IAimAssist{

    @Override
    public ChassisSpeeds handleAimAssist(ChassisSpeeds chassisSpeeds, Swerve swerve) {
        return chassisSpeeds;
    }

    @Override
    public String getName() {
        return "None";
    }
}
