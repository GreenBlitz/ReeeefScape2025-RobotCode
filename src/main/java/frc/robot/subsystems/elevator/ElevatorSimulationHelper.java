package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ElevatorSimulationHelper {

	public static Pose3d getFirstStagePoseFromHeight(double heightInMeters) {
		double height = Math.min(heightInMeters, ElevatorConstants.FIRST_STAGE_MAX_HEIGHT_METERS);

		return new Pose3d(new Translation3d(0, 0, height), new Rotation3d());
	}

	public static Pose3d getSecondStagePoseFromHeight(double heightInMeters) {
		return new Pose3d(new Translation3d(0, 0, heightInMeters), new Rotation3d());
	}


}
