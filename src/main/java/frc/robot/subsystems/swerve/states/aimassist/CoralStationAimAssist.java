package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;

import java.util.function.Supplier;

public class CoralStationAimAssist implements IRotationalAimAssist{

    private final CoralStation target;

    public CoralStationAimAssist(CoralStation coralStation) {
        target = coralStation;
    }

    @Override
    public Rotation2d getTargetHeading() {
        return Field.getCoralStationMiddle(target).getRotation();
    }

}
