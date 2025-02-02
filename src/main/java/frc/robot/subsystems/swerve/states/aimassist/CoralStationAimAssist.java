package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.CoralStation;


public class CoralStationAimAssist implements IRotationalAimAssist {

	private final CoralStation target;

	public CoralStationAimAssist(CoralStation coralStation) {
		target = coralStation;
	}

	@Override
	public Rotation2d getTargetHeading() {
		return Field.getCoralStationMiddle(target).getRotation();
	}

	@Override
	public String getName() {
		return "CoralStation";
	}
}
