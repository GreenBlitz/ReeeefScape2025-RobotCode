package frc.robot.superstructure;

import frc.robot.Robot;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import frc.robot.subsystems.swerve.Swerve;

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm());
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralInBack();
	}

	public boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	private boolean isReadyToScore(CoralScoringTarget coralScoringTarget) {
		return robot.getElevator().isAtPosition(coralScoringTarget.getElevatorTargetPositionMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& robot.getArm().isAtPosition(coralScoringTarget.getArmTargetPosition(), Tolerances.ARM_POSITION);
//		 && swerve.isattargetpos(reefLevel.getSwerveTargetPosition)
	}

}
