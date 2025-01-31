package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.utils.math.ToleranceMath;

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

	private boolean isReadyToScoreBranch(BranchLevel branchLevel, Branch branch) {
		return robot.getElevator().isAtPosition(branchLevel.getElevatorTargetPositionMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& robot.getArm().isAtPosition(branchLevel.getArmTargetPosition(), Tolerances.ARM_POSITION);
//			&& isAtPose(branchLevel.getTargetPosition(branch), Tolerances.BRANCH_SCORE_POSE, Tolerances.BRANCH_SCORE_VELOCITY_DEADBANDS);
	}

	private boolean isReadyToScoreL1() {
		return robot.getElevator().isAtPosition(ElevatorState.L1.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& robot.getArm().isAtPosition(ArmState.L1.getPosition(), Tolerances.ARM_POSITION);
//		    && isAtPose(reefLevel.getSwerveTargetPosition)
	}

	public boolean isAtPose(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds currentSpeeds, Pose2d tolerances, Pose2d deadbands) {
		boolean isAtX = MathUtil.isNear(targetPose.getX(), currentPose.getX(), tolerances.getX());
		boolean isAtY = MathUtil.isNear(targetPose.getY(), currentPose.getY(), tolerances.getY());
		boolean isAtHeading = ToleranceMath.isNearWrapped(targetPose.getRotation(), currentPose.getRotation(), tolerances.getRotation());
		boolean isStopping = SwerveMath.isStill(currentSpeeds, deadbands);
		return isAtX && isAtY && isAtHeading && isStopping;
	}

}
