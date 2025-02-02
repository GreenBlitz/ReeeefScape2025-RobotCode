package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefSide;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Supplier;

public class ReefAimAssist implements IRotationalAimAssist {

    private ReefSide target;

    public ReefAimAssist(ReefSide reefSide) {
        this.target = reefSide;
    }

    @Override
    public Rotation2d getTargetHeading() {
        return Field.getReefSideMiddle(target).getRotation();
    }
}
