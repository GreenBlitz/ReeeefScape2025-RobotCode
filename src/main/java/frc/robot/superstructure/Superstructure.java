package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final String logPath;

	private final Robot robot;
//    private final SwerveStateHandler swerveStateHandler;
//    private final ElevatorStateHandler elevatorStateHandler;
//    private final ArmStateHandler armStateHandler;
//    private final EndEffectorStateHandler endEffectorStateHandler;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		this.logPath = logPath;

		this.robot = robot;
//        this.swerve = new SwerveStateHandler(robot.getSwerve());
//        this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
//        this.armStateHandler = new ArmStateHandler(robot.getArm());
//        this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

	private boolean isCoralInEndEffector() {
		return robot.getEndEffector().isCoralInBack() && robot.getEndEffector().isCoralInFront();
	}

	private boolean isReadyToScoreL1() {
		// return isElevatorAtPos && isArmAtPos
		return true;
	}

	private boolean isReadyToScoreL2() {
		// return isElevatorAtPos && isArmAtPos
		return true;
	}

	private boolean isReadyToScoreL3() {
		// return isElevatorAtPos && isArmAtPos
		return true;
	}

	private boolean isReadyToScoreL4() {
		// return isElevatorAtPos && isArmAtPos
		return true;
	}

    //@formatter:off
    private Command setState(RobotState state) {
		return new ParallelCommandGroup(
            new InstantCommand(() -> currentState = state),
            switch (state) {
			    case IDLE -> idle();
		    	case INTAKE -> intake();
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
            //elevator.closed
            //endeffector.keep
        );
    }

    public Command intake(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    //elevator.feeder
                    //arm.feeder
                    //endeffector.intake
                ).until(this::isCoralInEndEffector),
                new ParallelCommandGroup(
                    //elevator.closed
                    //arm.closed
                    //endeffector.keep
                )
            )
            //swerve.aimassist.intake
        );
    }

    public Command l1(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    //elevator.l1
                    //arm.l1
                    //endeffector.keep
                ).until(this::isReadyToScoreL1),
                //endeffector.outtake.until(!isCoralIn),
                new ParallelCommandGroup(
                    //elevator.closed
                    //arm.closed
                    //endeffector.keep
                )
            )
            //swerve.aimassist.reef
        );
    }

    public Command l2(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    //elevator.l2
                    //arm.l2
                    //endeffector.keep
                ).until(this::isReadyToScoreL2),
                //endeffector.outtake.until(!isCoralIn),
                new ParallelCommandGroup(
                    //elevator.closed
                    //arm.closed
                    //endeffector.keep
                )
            )
            //swerve.aimassist.reef
        );
    }

    public Command l3(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    //elevator.l3
                    //arm.l3
                    //endeffector.keep
                ).until(this::isReadyToScoreL3),
                //endeffector.outtake.until(!isCoralIn),
                new ParallelCommandGroup(
                    //elevator.closed
                    //arm.closed
                    //endeffector.keep
                )
            )
            //swerve.aimassist.reef
        );
    }

    public Command l4(){
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    //elevator.l4
                    //arm.l4
                    //endeffector.keep
                ).until(this::isReadyToScoreL4),
                //endeffector.outtake.until(!isCoralIn),
                new ParallelCommandGroup(
                    //elevator.closed
                    //arm.closed
                    //endeffector.keep
                )
            )
            //swerve.aimassist.reef
        );
    }

    public Command preL1(){
        return new ParallelCommandGroup(
            //elevator.l1
            //arm.prel1
            //endeffector.keep
            //swerve.defaultdrive
        );
    }

    public Command preL2(){
        return new ParallelCommandGroup(
            //elevator.l2
            //arm.prel2
            //endeffector.keep
            //swerve.defaultdrive
        );
    }

    public Command preL3(){
        return new ParallelCommandGroup(
            //elevator.l3
            //arm.prel3
            //endeffector.keep
            //swerve.defaultdrive
        );
    }

    public Command preL4(){
        return new ParallelCommandGroup(
            //elevator.l4
            //arm.prel4
            //endeffector.keep
            //swerve.defaultdrive
        );
    }

    public Command outtake(){
        return new ParallelCommandGroup(
            //swerve.defaultdrive
            //elevator.outtake
            //arm.outtake
            //endeffector.outtake
        );
    }

    public Command alignReef(){
        return new ParallelCommandGroup(
            //swerve.aimassist.alignReef
            //elevator.closed
            //arm.closed
            //endeffector.keep
        );
    }
    //@formatter:on

}
