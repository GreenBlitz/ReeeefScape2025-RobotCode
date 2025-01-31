package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.enums.ReefBranch;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

import java.util.function.Function;

public enum BranchLevel {

	L2(
		ElevatorState.L2.getHeightMeters(),
		ArmState.L2.getPosition(),
		SuperstructureConstants.SCORING_POSE_DISTANCE_FROM_REEF_METERS
	),
	L3(
		ElevatorState.L3.getHeightMeters(),
		ArmState.L3.getPosition(),
		SuperstructureConstants.SCORING_POSE_DISTANCE_FROM_REEF_METERS
	),
	L4(
		ElevatorState.L4.getHeightMeters(),
		ArmState.L4.getPosition(),
		SuperstructureConstants.SCORING_POSE_DISTANCE_FROM_REEF_METERS
	);

	private final double elevatorTargetPositionMeters;
	private final Rotation2d armTargetPosition;
	private final Function<ReefBranch, Pose2d> targetPosition;

	BranchLevel(double elevatorTargetPositionMeters, Rotation2d armTargetPosition, double scoringPoseDistanceFromBranchMeters) {
		this.elevatorTargetPositionMeters = elevatorTargetPositionMeters;
		this.armTargetPosition = armTargetPosition;
		this.targetPosition = branch -> ScoringHelpers.getRobotScoringPose(branch, scoringPoseDistanceFromBranchMeters);
	}

	public double getElevatorTargetPositionMeters() {
		return elevatorTargetPositionMeters;
	}

	public Rotation2d getArmTargetPosition() {
		return armTargetPosition;
	}

	public Pose2d getTargetPosition(ReefBranch branch) {
		return targetPosition.apply(branch);
	}

}
