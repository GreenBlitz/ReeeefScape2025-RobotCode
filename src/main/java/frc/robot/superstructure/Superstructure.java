package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.SwerveState;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends GBSubsystem {

	public class ParallelRobotCommand extends ParallelCommandGroup {

		public ParallelRobotCommand(Command... commands) {
			super(commands);
			addRequirements(Superstructure.this);
		}

	}

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

	public void log() {
		Logger.recordOutput(getLogPath() + "/CurrentState", currentState);
	}

	public boolean isCoralFullyIn() {
		return robot.getEndEffector().isCoralInFront() && robot.getEndEffector().isCoralInBack();
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralInFront();
	}

	public boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	private boolean isReadyToScore(ReefLevel reefLevel) {
		return robot.getElevator().isAtPosition(reefLevel.getElevatorTargetPositionMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& robot.getArm().isAtPosition(reefLevel.getArmTargetPosition(), Tolerances.ARM_POSITION);
		// && swerve.isattargetpos(reefLevel.getSwerveTargetPosition)
	}

	//@formatter:off
    public Command setState(RobotState state) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			switch (state) {
				case IDLE -> idle();
				case FEEDER_INTAKE -> intake();
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
			}
		);
	}

    public Command idle(){
        return new ParallelRobotCommand(
            elevatorStateHandler.setState(ElevatorState.CLOSED),
            armStateHandler.setState(ArmState.CLOSED),
            endEffectorStateHandler.setState(EndEffectorState.KEEP),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
        );
    }

    public Command intake(){
        return new ParallelRobotCommand(
			new SequentialCommandGroup(
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE).until(this::isCoralIn), //TODO aim assist feeder
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
            elevatorStateHandler.setState(ElevatorState.FEEDER),
            armStateHandler.setState(ArmState.INTAKE),
            endEffectorStateHandler.setState(EndEffectorState.INTAKE)
        ).until(this::isCoralFullyIn);
    }

    public Command l1(){
		return new ParallelRobotCommand(
			new SequentialCommandGroup(
				new RunCommand(() -> {}).until(() -> isReadyToScore(ReefLevel.L1)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			),
			elevatorStateHandler.setState(ElevatorState.L1),
			armStateHandler.setState(ArmState.L1),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
		).until(this::isCoralOut);
    }

    public Command l2(){
		return new ParallelRobotCommand(
			new SequentialCommandGroup(
				new RunCommand(() -> {}).until(() -> isReadyToScore(ReefLevel.L2)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			),
			elevatorStateHandler.setState(ElevatorState.L2),
			armStateHandler.setState(ArmState.L2),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
		).until(this::isCoralOut);
    }

    public Command l3(){
		return new ParallelRobotCommand(
			new SequentialCommandGroup(
				new RunCommand(() -> {}).until(() -> isReadyToScore(ReefLevel.L3)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			),
			elevatorStateHandler.setState(ElevatorState.L3),
			armStateHandler.setState(ArmState.L3),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
		).until(this::isCoralOut);
    }

    public Command l4(){
		return new ParallelRobotCommand(
			new SequentialCommandGroup(
				new RunCommand(() -> {}).until(() -> isReadyToScore(ReefLevel.L4)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			),
			elevatorStateHandler.setState(ElevatorState.L4),
			armStateHandler.setState(ArmState.L4),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
		).until(this::isCoralOut);
    }

    public Command preL1(){
        return new ParallelRobotCommand(
			elevatorStateHandler.setState(ElevatorState.L1),
			armStateHandler.setState(ArmState.PRE_L1),
            endEffectorStateHandler.setState(EndEffectorState.KEEP),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
        );
    }

    public Command preL2(){
        return new ParallelRobotCommand(
            elevatorStateHandler.setState(ElevatorState.L2),
            armStateHandler.setState(ArmState.PRE_L2),
            endEffectorStateHandler.setState(EndEffectorState.KEEP),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
        );
    }

    public Command preL3(){
        return new ParallelRobotCommand(
            elevatorStateHandler.setState(ElevatorState.L3),
            armStateHandler.setState(ArmState.PRE_L3),
            endEffectorStateHandler.setState(EndEffectorState.KEEP),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
        );
    }

    public Command preL4(){
        return new ParallelRobotCommand(
            elevatorStateHandler.setState(ElevatorState.L4),
            armStateHandler.setState(ArmState.PRE_L4),
            endEffectorStateHandler.setState(EndEffectorState.KEEP),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO branch aim assist
		);
    }

    public Command outtake(){
        return new ParallelRobotCommand(
            elevatorStateHandler.setState(ElevatorState.OUTTAKE),
            armStateHandler.setState(ArmState.OUTTAKE),
            endEffectorStateHandler.setState(EndEffectorState.OUTTAKE),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		).until(this::isCoralOut);
    }

    public Command alignReef(){
        return new ParallelRobotCommand(
            elevatorStateHandler.setState(ElevatorState.CLOSED),
            armStateHandler.setState(ArmState.CLOSED),
            endEffectorStateHandler.setState(EndEffectorState.KEEP),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE) //TODO reef aim assist
		);
    }

    private Command endState() {
        return setState(RobotState.IDLE);
    }
    //@formatter:on

}
