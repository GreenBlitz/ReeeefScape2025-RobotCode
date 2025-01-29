package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.enums.ReefBranch;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

import java.util.function.Function;

public enum ReefLevel {

	L1(ElevatorState.L1.getHeightMeters(), ArmState.L1.getPosition(), branch -> new Pose2d()),
	L2(ElevatorState.L2.getHeightMeters(), ArmState.L2.getPosition(), branch -> new Pose2d()),
	L3(ElevatorState.L3.getHeightMeters(), ArmState.L3.getPosition(), branch -> new Pose2d()),
	L4(ElevatorState.L4.getHeightMeters(), ArmState.L4.getPosition(), branch -> new Pose2d());

	private final double elevatorTargetPositionMeters;
	private final Rotation2d armTargetPosition;
	private final Function<ReefBranch, Pose2d> swerveTargetPosition;

	ReefLevel(double elevatorTargetPositionMeters, Rotation2d armTargetPosition, Function<ReefBranch, Pose2d> swerveTargetPosition) {
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
