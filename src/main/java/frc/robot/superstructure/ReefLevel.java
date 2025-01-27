package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

public enum ReefLevel {

    L1(ElevatorState.L1.getHeightMeters(), ArmState.L1.getPosition(), new Pose2d()),
    L2(ElevatorState.L2.getHeightMeters(), ArmState.L2.getPosition(), new Pose2d()),
    L3(ElevatorState.L3.getHeightMeters(), ArmState.L3.getPosition(), new Pose2d()),
    L4(ElevatorState.L4.getHeightMeters(), ArmState.L4.getPosition(), new Pose2d());

    private final double elevatorTargetPositionMeters;
    private final Rotation2d armTargetPosition;
    private final Pose2d swerveTargetPosition;

    ReefLevel(double elevatorTargetPositionMeters, Rotation2d armTargetPosition, Pose2d swerveTargetPosition){
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

    public Pose2d getSwerveTargetPosition() {
        return swerveTargetPosition;
    }

}
