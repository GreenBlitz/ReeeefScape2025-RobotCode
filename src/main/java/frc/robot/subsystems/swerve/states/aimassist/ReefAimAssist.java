package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefSide;


public class ReefAimAssist implements IRotationalAimAssist {

	private final ReefSide target;

	public ReefAimAssist(ReefSide reefSide) {
		target = reefSide;
	}

	@Override
	public Rotation2d getTargetHeading() {
		return Field.getReefSideMiddle(target).getRotation();
	}

	@Override
	public String getName() {
		return "Reef";
	}
}
