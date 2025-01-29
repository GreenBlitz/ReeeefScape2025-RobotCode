package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm());
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());

		setDefaultCommand(new DeferredCommand(this::endState, Set.of(this)));
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralInFront() && robot.getEndEffector().isCoralInBack();
	}

	public boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	public boolean isAtTargetPose(Pose2d targetPose, IPoseEstimator poseEstimator){
		boolean isXAxisInTolerance = MathUtil.isNear(
				targetPose.getX(),
				poseEstimator.getEstimatedPose().getX(),
				Tolerances.CHASSIS_POSITION_METERS
		);
		boolean isYAxisInTolerance = MathUtil.isNear(
				targetPose.getY(),
				poseEstimator.getEstimatedPose().getY(),
				Tolerances.CHASSIS_POSITION_METERS
		);
		
		boolean isRotationInTolerance = MathUtil.isNear(
				targetPose.getRotation().getRotations(),
				poseEstimator.getEstimatedPose().getRotation().getRotations(),
				Tolerances.CHASSIS_ROTATION.getRotations()
		);
		
		return isXAxisInTolerance && isYAxisInTolerance && isRotationInTolerance;
	}
	
	private boolean isReadyToScore(CoralScoringTarget coralScoringTarget) {
		return robot.getElevator().isAtPosition(coralScoringTarget.getElevatorTargetPositionMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& robot.getArm().isAtPosition(coralScoringTarget.getArmTargetPosition(), Tolerances.ARM_POSITION);
//		 && swerve.isattargetpos(reefLevel.getSwerveTargetPosition)
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "/CurrentState", currentState);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case IDLE -> idle();
			case FEEDER_INTAKE -> feederIntake();
			case L1 -> l1();
			case L2 -> l2();
			case L3 -> l3();
			case L4 -> l4();
			case PRE_L1 -> preL1();
			case PRE_L2 -> preL2();
			case PRE_L3 -> preL3();
			case PRE_L4 -> preL4();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
		};
	}

	public Command idle() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.KEEP),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.IDLE
		);
	}

	public Command feederIntake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE).until(this::isCoralIn), // TODO aim assist feeder
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
				),
				elevatorStateHandler.setState(ElevatorState.FEEDER),
				armStateHandler.setState(ArmState.INTAKE),
				endEffectorStateHandler.setState(EndEffectorState.INTAKE)
			).until(this::isCoralIn),
			RobotState.FEEDER_INTAKE
		);
	}

	public Command l1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(CoralScoringTarget.L1)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L1),
				armStateHandler.setState(ArmState.L1),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			).until(this::isCoralOut),
			RobotState.L1
		);
	}

	public Command l2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(CoralScoringTarget.L2)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L2),
				armStateHandler.setState(ArmState.L2),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			).until(this::isCoralOut),
			RobotState.L2
		);
	}

	public Command l3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(CoralScoringTarget.L3)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L3),
				armStateHandler.setState(ArmState.L3),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			).until(this::isCoralOut),
			RobotState.L3
		);
	}

	public Command l4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(CoralScoringTarget.L4)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L4),
				armStateHandler.setState(ArmState.L4),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			).until(this::isCoralOut),
			RobotState.L4
		);
	}

	public Command preL1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L1),
				armStateHandler.setState(ArmState.PRE_L1),
				endEffectorStateHandler.setState(EndEffectorState.KEEP),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			),
			RobotState.PRE_L1
		);
	}

	public Command preL2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L2),
				armStateHandler.setState(ArmState.PRE_L2),
				endEffectorStateHandler.setState(EndEffectorState.KEEP),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			),
			RobotState.PRE_L2
		);
	}

	public Command preL3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L3),
				armStateHandler.setState(ArmState.PRE_L3),
				endEffectorStateHandler.setState(EndEffectorState.KEEP),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			),
			RobotState.PRE_L3
		);
	}

	public Command preL4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L4),
				armStateHandler.setState(ArmState.PRE_L4),
				endEffectorStateHandler.setState(EndEffectorState.KEEP),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO branch aim assist
			),
			RobotState.PRE_L4
		);
	}

	public Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.OUTTAKE),
				armStateHandler.setState(ArmState.OUTTAKE),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			).until(this::isCoralOut),
			RobotState.OUTTAKE
		);
	}

	public Command alignReef() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.KEEP),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) // TODO reef aim assist
			),
			RobotState.ALIGN_REEF
		);
	}

	private Command endState() {
		return setState(RobotState.IDLE);
	}
	
	public Command asSubsystemCommand(Command command, RobotState state) {
		command =  super.asSubsystemCommand(command, state.name());
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			command
		);
	}
}
