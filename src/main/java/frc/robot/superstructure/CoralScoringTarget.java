package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.enums.ReefBranch;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

import java.util.function.Function;

public enum CoralScoringTarget {

	L1(
		ElevatorState.L1.getHeightMeters(),
		ArmState.L1.getPosition(),
		branch -> ScoringHelpers.getRobotScoringPose(branch, SuperstructureConstants.ROBOT_SCORING_POSE_DISTANCE_FROM_BRANCH_METERS)
	),
	L2(
		ElevatorState.L2.getHeightMeters(),
		ArmState.L2.getPosition(),
		branch -> ScoringHelpers.getRobotScoringPose(branch, SuperstructureConstants.ROBOT_SCORING_POSE_DISTANCE_FROM_BRANCH_METERS)
	),
	L3(
		ElevatorState.L3.getHeightMeters(),
		ArmState.L3.getPosition(),
		branch -> ScoringHelpers.getRobotScoringPose(branch, SuperstructureConstants.ROBOT_SCORING_POSE_DISTANCE_FROM_BRANCH_METERS)
	),
	L4(
		ElevatorState.L4.getHeightMeters(),
		ArmState.L4.getPosition(),
		branch -> ScoringHelpers.getRobotScoringPose(branch, SuperstructureConstants.ROBOT_SCORING_POSE_DISTANCE_FROM_BRANCH_METERS)
	);

	private final double elevatorTargetPositionMeters;
	private final Rotation2d armTargetPosition;
	private final Function<ReefBranch, Pose2d> swerveTargetPosition;

	CoralScoringTarget(double elevatorTargetPositionMeters, Rotation2d armTargetPosition, Function<ReefBranch, Pose2d> swerveTargetPosition) {
		this.elevatorTargetPositionMeters = elevatorTargetPositionMeters;
		this.armTargetPosition = armTargetPosition;
		this.swerveTargetPosition = swerveTargetPosition;
	}

	public double getElevatorTargetPositionMeters() {
		return elevatorTargetPositionMeters;
	}

	public Rotation2d getArmTargetPosition() {
		return armTargetPosition;
	}

	public Pose2d getSwerveTargetPosition(ReefBranch branch) {
		return swerveTargetPosition.apply(branch);
	}

}
