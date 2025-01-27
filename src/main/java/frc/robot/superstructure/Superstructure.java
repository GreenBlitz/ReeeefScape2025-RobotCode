package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	// private final SwerveStateHandler swerveStateHandler;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);

		this.robot = robot;
//        this.swerve = new SwerveStateHandler(robot.getSwerve());
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm());
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());

		Command defaultCommand = endState(getCurrentState()).asProxy();
		defaultCommand.addRequirements(this);
		this.setDefaultCommand(defaultCommand);
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "/CurrentState", currentState);
	}

	private boolean isCoralIn() {
		return robot.getEndEffector().isCoralInFront() && robot.getEndEffector().isCoralInBack();
	}

	private boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	private boolean isReadyToScore(ReefLevel reefLevel) {
		return robot.getElevator().isAtPosition(reefLevel.getElevatorTargetPositionMeters(), Tolerances.ELEVATOR_HEIGHT__METERS)
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
        return new ParallelCommandGroup(
            //swerve.defaultDrive
            elevatorStateHandler.setState(ElevatorState.CLOSED),
            armStateHandler.setState(ArmState.CLOSED),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
        );
    }

    public Command intake(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.FEEDER),
            armStateHandler.setState(ArmState.INTAKE),
            endEffectorStateHandler.setState(EndEffectorState.INTAKE)
            //swerve.aimassist.intake
        ).until(this::isCoralIn);
    }

    public Command l1(){
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				preL1().until(() -> isReadyToScore(ReefLevel.L1)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			)
			//swerve.aimassist.reef
		).until(this::isCoralOut);
    }

    public Command l2(){
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				preL2().until(() -> isReadyToScore(ReefLevel.L2)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			)
			//swerve.aimassist.reef
		).until(this::isCoralOut);
    }

    public Command l3(){
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				preL3().until(() -> isReadyToScore(ReefLevel.L3)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			)
			//swerve.aimassist.reef
		).until(this::isCoralOut);
    }

    public Command l4(){
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				preL4().until(() -> isReadyToScore(ReefLevel.L4)),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			)
			//swerve.aimassist.reef
		).until(this::isCoralOut);
    }

    public Command preL1(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L1),
            armStateHandler.setState(ArmState.PRE_L1),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.aimassist.reef
        );
    }

    public Command preL2(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L2),
            armStateHandler.setState(ArmState.PRE_L2),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.aimassist.reef
        );
    }

    public Command preL3(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L3),
            armStateHandler.setState(ArmState.PRE_L3),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.aimassist.reef
        );
    }

    public Command preL4(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L4),
            armStateHandler.setState(ArmState.PRE_L4),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.aimassist.reef
        );
    }

    public Command outtake(){
        return new ParallelCommandGroup(
            //swerve.defaultdrive
            elevatorStateHandler.setState(ElevatorState.OUTTAKE),
            armStateHandler.setState(ArmState.OUTTAKE),
            endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
        );
    }

    public Command alignReef(){
        return new ParallelCommandGroup(
            //swerve.aimassist.alignReef
            elevatorStateHandler.setState(ElevatorState.CLOSED),
            armStateHandler.setState(ArmState.CLOSED),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
        );
    }

    private  Command endState(RobotState state) {
        return setState(RobotState.IDLE);
    }
    //@formatter:on

}
