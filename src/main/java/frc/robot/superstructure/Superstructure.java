package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final String logPath;

	private final Robot robot;
//    private final SwerveStateHandler swerveStateHandler;
	private final ElevatorStateHandler elevatorStateHandler;
    private final ArmStateHandler armStateHandler;
    private final EndEffectorStateHandler endEffectorStateHandler;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		this.logPath = logPath;

		this.robot = robot;
//        this.swerve = new SwerveStateHandler(robot.getSwerve());
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
        this.armStateHandler = new ArmStateHandler(robot.getArm());
        this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void log() {
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

	private boolean isCoralFullyIn() {
		return robot.getEndEffector().isCoralInBack() && robot.getEndEffector().isCoralInFront();
	}

    private boolean isCoralIn(){
        return robot.getEndEffector().isCoralInFront();
    }

	private boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	private boolean isReadyToScoreL1() {
		return robot.getElevator().isAtPosition(ElevatorState.L1.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_TOLERANCE_METERS) && robot.getArm().isAtPosition(ArmState.L1.getPosition(), Tolerances.ARM_ANGLE_TOLERANCE);
	}

	private boolean isReadyToScoreL2() {
		return robot.getElevator().isAtPosition(ElevatorState.L2.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_TOLERANCE_METERS) && robot.getArm().isAtPosition(ArmState.L2.getPosition(), Tolerances.ARM_ANGLE_TOLERANCE);
	}

	private boolean isReadyToScoreL3() {
		return robot.getElevator().isAtPosition(ElevatorState.L3.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_TOLERANCE_METERS) && robot.getArm().isAtPosition(ArmState.L3.getPosition(), Tolerances.ARM_ANGLE_TOLERANCE);
	}

	private boolean isReadyToScoreL4() {
		return robot.getElevator().isAtPosition(ElevatorState.L4.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_TOLERANCE_METERS) && robot.getArm().isAtPosition(ArmState.L4.getPosition(), Tolerances.ARM_ANGLE_TOLERANCE);
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
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevatorStateHandler.setState(ElevatorState.FEEDER),
                    armStateHandler.setState(ArmState.INTAKE),
                    endEffectorStateHandler.setState(EndEffectorState.INTAKE)
                ).until(this::isCoralIn)
            )
            //swerve.aimassist.intake
        );
    }

    public Command l1(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevatorStateHandler.setState(ElevatorState.L1),
                    armStateHandler.setState(ArmState.L1),
                    endEffectorStateHandler.setState(EndEffectorState.KEEP)
                ).until(this::isReadyToScoreL1),
                endEffectorStateHandler.setState(EndEffectorState.OUTTAKE).until(() -> !isCoralFullyIn())
            )
            //swerve.aimassist.reef
        );
    }

    public Command l2(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevatorStateHandler.setState(ElevatorState.L2),
                    armStateHandler.setState(ArmState.L2),
                    endEffectorStateHandler.setState(EndEffectorState.KEEP)
                ).until(this::isReadyToScoreL2),
                endEffectorStateHandler.setState(EndEffectorState.OUTTAKE).until(() -> !isCoralFullyIn()),
                new ParallelCommandGroup(
                    elevatorStateHandler.setState(ElevatorState.CLOSED),
                    armStateHandler.setState(ArmState.CLOSED),
                    endEffectorStateHandler.setState(EndEffectorState.KEEP)
                )
            )
            //swerve.aimassist.reef
        );
    }

    public Command l3(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevatorStateHandler.setState(ElevatorState.L3),
                    armStateHandler.setState(ArmState.L3),
                    endEffectorStateHandler.setState(EndEffectorState.KEEP)
                ).until(this::isReadyToScoreL3),
                endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
            )
            //swerve.aimassist.reef
        );
    }

    public Command l4(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevatorStateHandler.setState(ElevatorState.L4),
                    armStateHandler.setState(ArmState.L4),
                    endEffectorStateHandler.setState(EndEffectorState.KEEP)
                ).until(this::isReadyToScoreL4),
                endEffectorStateHandler.setState(EndEffectorState.OUTTAKE).until(() -> !isCoralFullyIn())
            )
            //swerve.aimassist.reef
        );
    }

    public Command preL1(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L1),
            armStateHandler.setState(ArmState.PRE_L1),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.defaultdrive
        );
    }

    public Command preL2(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L2),
            armStateHandler.setState(ArmState.PRE_L2),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.defaultdrive
        );
    }

    public Command preL3(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L3),
            armStateHandler.setState(ArmState.PRE_L3),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.defaultdrive
        );
    }

    public Command preL4(){
        return new ParallelCommandGroup(
            elevatorStateHandler.setState(ElevatorState.L4),
            armStateHandler.setState(ArmState.PRE_L4),
            endEffectorStateHandler.setState(EndEffectorState.KEEP)
            //swerve.defaultdrive
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

    public void endState(RobotState state) {
        setState(RobotState.IDLE);
    }
    //@formatter:on

}
