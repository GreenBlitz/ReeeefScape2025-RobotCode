package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeStateHandler;
import frc.robot.subsystems.algaeIntake.pivot.PivotStateHandler;
import frc.robot.subsystems.algaeIntake.rollers.RollersStateHandler;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbStateHandler;
import frc.robot.subsystems.climb.lifter.LifterConstants;
import frc.robot.subsystems.climb.lifter.LifterState;
import frc.robot.subsystems.climb.lifter.LifterStateHandler;
import frc.robot.subsystems.climb.solenoid.SolenoidStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;
	private final ClimbStateHandler climbStateHandler;
	private final AlgaeIntakeStateHandler algaeIntakeStateHandler;

	private SuperstructureState currentState;
	public boolean driverIsCoralInOverride;
	public boolean driverIsAlgaeInEndEffectorOverride;
	public boolean driverIsAlgaeInAlgaeIntakeOverride;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm(), this::getDistanceToReef);
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector(), this);
		this.climbStateHandler = new ClimbStateHandler(new SolenoidStateHandler(robot.getSolenoid()), new LifterStateHandler(robot.getLifter()));
		this.algaeIntakeStateHandler = new AlgaeIntakeStateHandler(
			new PivotStateHandler(robot.getPivot()),
			new RollersStateHandler(robot.getRollers())
		);

		this.currentState = SuperstructureState.STAY_IN_PLACE;
		this.driverIsCoralInOverride = false;
		this.driverIsAlgaeInEndEffectorOverride = false;
		this.driverIsAlgaeInAlgaeIntakeOverride = false;
		setDefaultCommand(
			new DeferredCommand(
				() -> endState(currentState),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			)
		);
	}

	public ElevatorStateHandler getElevatorStateHandler() {
		return elevatorStateHandler;
	}

	public ArmStateHandler getArmStateHandler() {
		return armStateHandler;
	}

	public EndEffectorStateHandler getEndEffectorStateHandler() {
		return endEffectorStateHandler;
	}

	public ClimbStateHandler getClimbStateHandler() {
		return climbStateHandler;
	}

	public AlgaeIntakeStateHandler getAlgaeIntakeStateHandler() {
		return algaeIntakeStateHandler;
	}

	public SuperstructureState getCurrentState() {
		return currentState;
	}

	public Rotation2d getArmReversedSoftLimitByElevator() {
		return robot.getElevator().getElevatorPositionMeters() >= ArmConstants.ELEVATOR_HEIGHT_METERS_TO_CHANGE_SOFT_LIMIT
			? ArmConstants.ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT
			: ArmConstants.ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT;
	}

	public double getDistanceToReef() {
		return robot.getPoseEstimator()
			.getEstimatedPose()
			.getTranslation()
			.getDistance(Field.getCoralPlacement(ScoringHelpers.getTargetBranch(), true));
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralIn() || driverIsCoralInOverride;
	}

	public boolean isAlgaeInEndEffector() {
		return robot.getEndEffector().isAlgaeIn() || driverIsAlgaeInEndEffectorOverride;
	}

	public boolean isAlgaeInAlgaeIntake() {
		return algaeIntakeStateHandler.isAlgaeIn() || driverIsAlgaeInAlgaeIntakeOverride;
	}

	public boolean isClosed() {
		return robot.getElevator().isAtPosition(ElevatorState.CLOSED.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.CLOSED
			&& armStateHandler.isAtState(ArmState.CLOSED)
			&& armStateHandler.getCurrentState() == ArmState.CLOSED;
	}

	public boolean isPreNetReady() {
		return robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_TO_START_THROW_NET)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.NET
			&& armStateHandler.isAtState(ArmState.PRE_NET, Tolerances.ALGAE_RELEASE_ARM_POSITION);
	}

	public boolean isPreScoreReady() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		ArmState targetArmState = targetScoreLevel == ScoreLevel.L4 ? targetScoreLevel.getArmScore() : targetScoreLevel.getArmPreScore();

		return robot.getElevator().isAtPosition(targetScoreLevel.getElevatorPreScore().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == targetScoreLevel.getElevatorPreScore()
			&& armStateHandler.isAtState(targetArmState)
			&& armStateHandler.getCurrentState() == targetArmState;
	}

	public boolean isReadyToScore() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		return robot.getElevator().isAtPosition(targetScoreLevel.getElevatorScore().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == targetScoreLevel.getElevatorScore()
			&& armStateHandler.isAtState(targetScoreLevel.getArmScore())
			&& armStateHandler.getCurrentState() == targetScoreLevel.getArmScore();
	}

	public boolean isReadyToOuttakeAlgae() {
		return robot.getElevator().isAtPosition(ElevatorState.ALGAE_OUTTAKE.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.ALGAE_OUTTAKE
			&& armStateHandler.isAtState(ArmState.ALGAE_OUTTAKE, Tolerances.ALGAE_RELEASE_ARM_POSITION);
	}


	public boolean isReadyToProcessor() {
		return robot.getElevator().isAtPosition(ElevatorState.PROCESSOR_OUTTAKE.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.PROCESSOR_OUTTAKE
			&& armStateHandler.isAtState(ArmState.PROCESSOR_OUTTAKE, Tolerances.ALGAE_RELEASE_ARM_POSITION);
	}

	public boolean isReadyForNetRelease() {
		return robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_TO_RELEASE_NET)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.NET
			&& robot.getArm().isBehindPosition(StateMachineConstants.ARM_POSITION_TO_RELEASE_NET)
			&& armStateHandler.getCurrentState() == ArmState.NET;
	}

	@Override
	protected void subsystemPeriodic() {
		algaeIntakeStateHandler.updateAlgaeSensor(robot);
		log();
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/ElevatorState", elevatorStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/ArmState", armStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/EndEffectorState", endEffectorStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/ClimbState", climbStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/AlgaeIntakeState", algaeIntakeStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/IsAlgaeInIntake", isAlgaeInAlgaeIntake());
	}

	public Command idle() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.STOP),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			SuperstructureState.IDLE
		);
	}

	public Command stayInPlace() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.STAY_IN_PLACE),
				armStateHandler.setState(ArmState.STAY_IN_PLACE),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.STOP),
				algaeIntakeStateHandler.setState(AlgaeIntakeState.STAY_IN_PLACE)
			),
			SuperstructureState.STAY_IN_PLACE
		);
	}

	public Command intake() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.INTAKE),
					armStateHandler.setState(ArmState.INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.CORAL_INTAKE),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				).until(this::isCoralIn),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.INTAKE),
					armStateHandler.setState(ArmState.INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.CORAL_INTAKE),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				).withTimeout(StateMachineConstants.INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS)
			),
			SuperstructureState.INTAKE
		);
	}

	public Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.STAY_IN_PLACE),
				armStateHandler.setState(ArmState.STAY_IN_PLACE),
				endEffectorStateHandler.setState(EndEffectorState.CORAL_OUTTAKE),
				climbStateHandler.setState(ClimbState.STOP),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			).until(() -> !isCoralIn()),
			SuperstructureState.OUTTAKE
		);
	}

	public Command armPreScore() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorWhileDrive()),
					armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.ARM_PRE_SCORE
		);
	}

	private Command genericPreScore() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorPreScore()),
					armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.PRE_SCORE
		);
	}

	private Command l4PreScore() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					new SequentialCommandGroup(
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmPreScore())
							.until(() -> robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_TO_MOVE_ARM_TO_SCORE_L4)),
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore())
					),
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.SCORE_WITHOUT_RELEASE
		);
	}

	public Command preScore() {
		return new DeferredCommand(() -> switch (ScoringHelpers.targetScoreLevel) {
			case L4 -> l4PreScore();
			case L1, L2, L3 -> genericPreScore();
		},
			Set.of(
				this,
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)
		);
	}

	public Command scoreWithoutRelease() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorScore()),
					armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.SCORE_WITHOUT_RELEASE
		);
	}

	public Command scoreWithRelease() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new SequentialCommandGroup(
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorScore()),
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore()),
						endEffectorStateHandler.setState(ScoringHelpers.targetScoreLevel.getEndEffectorScore()),
						climbStateHandler.setState(ClimbState.STOP),
						algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
					).until(() -> !isCoralIn()),
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorScore()),
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore()),
						endEffectorStateHandler.setState(ScoringHelpers.targetScoreLevel.getEndEffectorScore()),
						climbStateHandler.setState(ClimbState.STOP),
						algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
					).withTimeout(StateMachineConstants.SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.SCORE
		);
	}

	public Command softCloseL4() {
		return softClose("L4", ArmState.MID_WAY_CLOSE, ArmState.CLOSED, ElevatorState.L4, ElevatorState.CLOSED, 0.8, Rotation2d.fromDegrees(45));
	}

	public Command softCloseNet() {
		return softClose(
			"Net",
			ArmState.MID_WAY_CLOSE,
			ArmState.CLOSED,
			ElevatorState.NET,
			ElevatorState.CLOSED,
			0.6,
			Rotation2d.fromDegrees(45)
		);
	}

	public Command softCloseNetToAlgaeRemove() {
		return new DeferredCommand(
			() -> softClose(
				"Net",
				ArmState.MID_WAY_CLOSE,
				ScoringHelpers.getAlgaeRemoveLevel().getArmState(),
				ElevatorState.NET,
				ScoringHelpers.getAlgaeRemoveLevel().getElevatorState(),
				0.6,
				Rotation2d.fromDegrees(45)
			),
			Set.of(
				this,
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)

		);
	}

	private Command softClose(
		String name,
		ArmState notTouchingField,
		ArmState closed,
		ElevatorState starting,
		ElevatorState ending,
		double elevatorHeightToCloseArm,
		Rotation2d armPositionToCloseElevator
	) {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					new ParallelCommandGroup(armStateHandler.setState(notTouchingField), elevatorStateHandler.setState(starting))
						.until(() -> robot.getArm().isPastPosition(armPositionToCloseElevator)),
					new ParallelCommandGroup(armStateHandler.setState(notTouchingField), elevatorStateHandler.setState(ending))
						.until(() -> !robot.getElevator().isPastPosition(elevatorHeightToCloseArm)),
					new ParallelDeadlineGroup(armStateHandler.setState(closed), elevatorStateHandler.setState(ending)).until(
						() -> armStateHandler.isAtState(closed, Tolerances.ARM_POSITION)
							&& elevatorStateHandler.isAtState(ending, Tolerances.ELEVATOR_HEIGHT_METERS)
					)
				),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.STOP),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			"Soft Close " + name
		);
	}

	public Command algaeRemove() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new SequentialCommandGroup(
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getElevatorState()),
						armStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getArmState()),
						endEffectorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getEndEffectorState()),
						climbStateHandler.setState(ClimbState.STOP),
						algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
					)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.ALGAE_REMOVE
		);
	}

	public Command algaeRemoveWithKeepRollers() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new SequentialCommandGroup(
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getElevatorState()),
						armStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getArmState()),
						endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
						climbStateHandler.setState(ClimbState.STOP)
					)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.ALGAE_REMOVE
		);
	}

	public Command processorScore() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.PROCESSOR_OUTTAKE),
					armStateHandler.setState(ArmState.PROCESSOR_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				).until(this::isReadyToProcessor),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.PROCESSOR_OUTTAKE),
					armStateHandler.setState(ArmState.PROCESSOR_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.PROCESSOR_OUTTAKE),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				)
			),
//					.until(() -> !isAlgaeIn()),
			SuperstructureState.PROCESSOR_OUTTAKE
		);
	}

	public Command processorWithoutRelease() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.PROCESSOR_OUTTAKE),
					armStateHandler.setState(ArmState.PROCESSOR_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP)
				)
			),
			SuperstructureState.PROCESSOR_OUTTAKE
		);
	}

	public Command algaeOuttakeFromEndEffector() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.ALGAE_OUTTAKE),
					armStateHandler.setState(ArmState.ALGAE_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				).until(this::isReadyToOuttakeAlgae),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.ALGAE_OUTTAKE),
					armStateHandler.setState(ArmState.ALGAE_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.ALGAE_OUTTAKE),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				)
			),
			SuperstructureState.ALGAE_OUTTAKE_FROM_END_EFFECTOR
		);
	}

	public Command algaeIntake() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new SequentialCommandGroup(
					new ParallelCommandGroup(
						elevatorStateHandler
							.setState(robot.getEndEffector().isCoralIn() ? ElevatorState.CLOSED : ElevatorState.TRANSFER_ALGAE_FROM_INTAKE),
						armStateHandler.setState(robot.getEndEffector().isCoralIn() ? ArmState.CLOSED : ArmState.TRANSFER_ALGAE_FROM_INTAKE),
						endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
						climbStateHandler.setState(ClimbState.STOP),
						algaeIntakeStateHandler.setState(AlgaeIntakeState.INTAKE)
					).until(this::isAlgaeInAlgaeIntake),
					new ParallelCommandGroup(
						elevatorStateHandler
							.setState(robot.getEndEffector().isCoralIn() ? ElevatorState.CLOSED : ElevatorState.TRANSFER_ALGAE_FROM_INTAKE),
						armStateHandler.setState(robot.getEndEffector().isCoralIn() ? ArmState.CLOSED : ArmState.TRANSFER_ALGAE_FROM_INTAKE),
						endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
						climbStateHandler.setState(ClimbState.STOP),
						algaeIntakeStateHandler.setState(AlgaeIntakeState.INTAKE)
					).withTimeout(StateMachineConstants.ALGAE_INTAKE_TIME_AFTER_SENSOR_SECONDS)
				),
				Set.of(
					this,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			SuperstructureState.ALGAE_FLOOR_INTAKE
		);
	}

	public Command algaeOuttakeFromIntake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					algaeIntakeStateHandler.setState(AlgaeIntakeState.OUTTAKE_WITH_RELEASE).until(() -> !isAlgaeInAlgaeIntake()),
					algaeIntakeStateHandler.setState(AlgaeIntakeState.OUTTAKE_WITH_RELEASE)
						.withTimeout(StateMachineConstants.ALGAE_OUTTAKE_FROM_INTAKE_TIME_AFTER_SENSOR_SECONDS)
				),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.CLOSED),
					armStateHandler.setState(ArmState.CLOSED),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.CLOSE)
				)
			),
			SuperstructureState.ALGAE_OUTTAKE_FROM_INTAKE
		);
	}

	public Command transferAlgaeFromIntakeToEndEffector() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.TRANSFER_ALGAE_FROM_INTAKE),
					armStateHandler.setState(ArmState.TRANSFER_ALGAE_FROM_INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.setState(AlgaeIntakeState.PUSH_ALGAE_OUT)
						.withTimeout(StateMachineConstants.PUSH_ALGAE_OUT_TIME_SECONDS)
						.andThen(algaeIntakeStateHandler.setState(AlgaeIntakeState.HOLD_ALGAE))
				).until(
					() -> armStateHandler.isAtState(ArmState.TRANSFER_ALGAE_FROM_INTAKE)
						&& elevatorStateHandler.isAtState(ElevatorState.TRANSFER_ALGAE_FROM_INTAKE)
						&& algaeIntakeStateHandler.getCurrentState() == AlgaeIntakeState.HOLD_ALGAE
				),
				new ParallelCommandGroup(
					new SequentialCommandGroup(
						endEffectorStateHandler.setState(EndEffectorState.TRANSFER_ALGAE_FROM_INTAKE).withTimeout(0.8),
						endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
					),
					elevatorStateHandler.setState(ElevatorState.TRANSFER_ALGAE_FROM_INTAKE),
					armStateHandler.setState(ArmState.TRANSFER_ALGAE_FROM_INTAKE),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.setState(AlgaeIntakeState.TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE)
				).until(() -> algaeIntakeStateHandler.isAtState(AlgaeIntakeState.TRANSFER_TO_END_EFFECTOR_WITHOUT_RELEASE))
					.withTimeout(StateMachineConstants.ALGAE_TRANSFER_TO_END_EFFECTOR_TIME_SECONDS),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.TRANSFER_ALGAE_FROM_INTAKE_RISE),
					armStateHandler.setState(ArmState.HOLD_ALGAE),
					new SequentialCommandGroup(
						endEffectorStateHandler.setState(EndEffectorState.TRANSFER_ALGAE_FROM_INTAKE).withTimeout(0.4),
						endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
					),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.setState(AlgaeIntakeState.TRANSFER_TO_END_EFFECTOR_WITH_RELEASE)

				).until(() -> robot.getArm().isBehindPosition(Rotation2d.fromDegrees(130)))
			),
			SuperstructureState.TRANSFER_ALGAE_TO_END_EFFECTOR
		);
	}

	public Command preNet() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.NET),
				armStateHandler.setState(ArmState.PRE_NET),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.CLOSE),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			SuperstructureState.PRE_NET
		);
	}

	public Command netWithRelease() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					endEffectorStateHandler.setState(EndEffectorState.NET_OUTTAKE).withTimeout(StateMachineConstants.NET_OUTTAKE_TIME_SECONDS)
				),
				elevatorStateHandler.setState(ElevatorState.NET),
				new SequentialCommandGroup(
					armStateHandler.setState(ArmState.PRE_NET).until(this::isPreNetReady),
					armStateHandler.setState(ArmState.NET)
				),
				climbStateHandler.setState(ClimbState.STOP),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			SuperstructureState.NET
		);
	}

	public Command preClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new ParallelCommandGroup(armStateHandler.setState(ArmState.CLIMB), climbStateHandler.setState(ClimbState.STOP))
						.until(() -> !robot.getArm().isPastPosition(StateMachineConstants.ARM_POSITION_TO_DEPLOY_LIFTER)),
					new ParallelCommandGroup(climbStateHandler.setState(ClimbState.DEPLOY), armStateHandler.setState(ArmState.CLIMB))
				),
				elevatorStateHandler.setState(ElevatorState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			SuperstructureState.PRE_CLIMB
		);
	}

	public Command climbWithoutLimitSwitch() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLIMB),
				armStateHandler.setState(ArmState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				climbStateHandler.setState(ClimbState.CLIMB_WITHOUT_LIMIT_SWITCH),
				algaeIntakeStateHandler.setState(AlgaeIntakeState.CLIMB)
			),
			SuperstructureState.CLIMB_WITHOUT_LIMIT_SWITCH
		);
	}

	public Command climbWithLimitSwitch() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					new ParallelCommandGroup(
						climbStateHandler.setState(ClimbState.CLIMB_WITH_LIMIT_SWITCH),
						algaeIntakeStateHandler.setState(AlgaeIntakeState.CLOSED)
					).until(
						() -> robot.getLifter()
							.isLower(
								Rotation2d.fromDegrees(
									climbStateHandler.getClimbPositionWithLimitSwitch().getDegrees()
										- LifterConstants.CLIMB_OFFSET_AFTER_LIMIT_SWITCH.getDegrees()
								)
							)
					).until(() -> robot.getLifter().isLower(LifterConstants.MINIMUM_CLIMB_POSITION)),
					new ParallelCommandGroup(
						climbStateHandler.setState(ClimbState.STOP),
						algaeIntakeStateHandler.setState(AlgaeIntakeState.CLIMB)
					)
				),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.CLIMB),
					armStateHandler.setState(ArmState.CLIMB),
					endEffectorStateHandler.setState(EndEffectorState.STOP)
				)
			),
			SuperstructureState.CLIMB_WITH_LIMIT_SWITCH
		);
	}

	public Command manualClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLIMB),
				armStateHandler.setState(ArmState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				climbStateHandler.setState(ClimbState.MANUAL_CLIMB),
				algaeIntakeStateHandler.setState(AlgaeIntakeState.CLIMB)
			),
			SuperstructureState.MANUAL_CLIMB
		);
	}

	public Command exitClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLIMB),
				armStateHandler.setState(ArmState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				climbStateHandler.setState(ClimbState.EXIT_CLIMB),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			SuperstructureState.EXIT_CLIMB
		);
	}

	public Command stopClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLIMB),
				armStateHandler.setState(ArmState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				climbStateHandler.setState(ClimbState.STOP),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			),
			SuperstructureState.STOP_CLIMB
		);
	}

	public Command closeClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new ParallelCommandGroup(armStateHandler.setState(ArmState.START_GAME), climbStateHandler.setState(ClimbState.STOP))
						.until(() -> robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_TO_CLOSE_CLIMB)),
					new ParallelCommandGroup(armStateHandler.setState(ArmState.PRE_L4), climbStateHandler.setState(ClimbState.CLOSE))
				),
				elevatorStateHandler.setState(ElevatorState.WHILE_DRIVE_L4),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
			).until(() -> robot.getLifter().isLower(LifterState.CLOSE.getTargetPosition())),
			SuperstructureState.CLOSE_CLIMB
		);
	}

	public Command holdAlgae() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.HOLD_ALGAE),
					armStateHandler.setState(ArmState.HOLD_ALGAE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle(driverIsAlgaeInAlgaeIntakeOverride)
				)
			),
			SuperstructureState.HOLD_ALGAE
		);
	}

	public Command elevatorOpening() {
		return asSubsystemCommand(elevatorStateHandler.setState(ElevatorState.OPENING_HEIGHT), SuperstructureState.ELEVATOR_OPENING)
			.until(() -> robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_FOR_OPENING));
	}

	private Command asSubsystemCommand(Command command, SuperstructureState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(SuperstructureState state) {
		return switch (state) {
			case STAY_IN_PLACE, OUTTAKE -> stayInPlace();
			case TRANSFER_ALGAE_TO_END_EFFECTOR, HOLD_ALGAE -> holdAlgae();
			case
				INTAKE,
				IDLE,
				ALGAE_REMOVE,
				ALGAE_OUTTAKE_FROM_END_EFFECTOR,
				PROCESSOR_OUTTAKE,
				PRE_NET,
				ALGAE_FLOOR_INTAKE,
				ALGAE_OUTTAKE_FROM_INTAKE ->
				idle();
			case NET -> softCloseNet().andThen(idle());
			case ARM_PRE_SCORE, CLOSE_CLIMB -> armPreScore();
			case PRE_SCORE, SCORE, SCORE_WITHOUT_RELEASE -> preScore();
			case PRE_CLIMB -> preClimb();
			case CLIMB_WITHOUT_LIMIT_SWITCH, CLIMB_WITH_LIMIT_SWITCH, MANUAL_CLIMB, EXIT_CLIMB, STOP_CLIMB -> stopClimb();
			case ELEVATOR_OPENING -> elevatorOpening();
		};
	}

}
