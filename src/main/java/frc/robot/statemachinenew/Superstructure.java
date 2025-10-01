package frc.robot.statemachinenew;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeStateHandler;
import frc.robot.subsystems.algaeIntake.pivot.PivotStateHandler;
import frc.robot.subsystems.algaeIntake.rollers.RollersStateHandler;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbStateHandler;
import frc.robot.subsystems.climb.lifter.LifterStateHandler;
import frc.robot.subsystems.climb.solenoid.SolenoidStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.BooleanSupplier;

public class Superstructure extends GBSubsystem {

    // (intake algae -> hold algae) must be transfer (they cannot be neighbors)
    // (net -> close) must be soft close (they cannot be neighbors)
    // (L4 -> close) must be soft close (they cannot be neighbors)
    // (while drive l4 -> l4) must be pre l4 (they cannot be neighbors)


    private final BooleanSupplier releaseReef;


	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;
	private final ClimbStateHandler climbStateHandler;
	private final AlgaeIntakeStateHandler algaeIntakeStateHandler;
	private final Set<Subsystem> subsystems;

	public Superstructure(String logPath, Robot robot, Targets targets) {
		super(logPath);

		this.subsystems = Set.of(
			robot.getElevator(),
			robot.getArm(),
			robot.getEndEffector(),
			robot.getLifter(),
			robot.getSolenoid(),
			robot.getPivot(),
			robot.getRollers()
		);

        Trigger transfer = new Trigger(() -> isAlgaeInIntake() && isAlgaeWantedUp).onTrue(intake to hold);

		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm(), targets::getDistanceToReef);
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());
		this.climbStateHandler = new ClimbStateHandler(new SolenoidStateHandler(robot.getSolenoid()), new LifterStateHandler(robot.getLifter()));
		this.algaeIntakeStateHandler = new AlgaeIntakeStateHandler(
			new PivotStateHandler(robot.getPivot()),
			new RollersStateHandler(robot.getRollers())
		);
	}

    public boolean isReadyToNet() {
        return elevatorStateHandler.isAtState(ElevatorState.NET) && armStateHandler.isAtState(ArmState.NET);
    }

	@Override
	protected void subsystemPeriodic() {
		algaeIntakeStateHandler.updateAlgaeSensor();
		log();
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/ElevatorState", elevatorStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/ArmState", armStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/EndEffectorState", endEffectorStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/ClimbState", climbStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/AlgaeIntakeState", algaeIntakeStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/IsAlgaeInIntake", isAlgaeInIntake());
	}

	public boolean isCoralIn() {
		return endEffectorStateHandler.isCoralIn();
	}

	public boolean isAlgaeInIntake() {
		return algaeIntakeStateHandler.isAlgaeIn();
	}


	public Command close() {
		return asSubsystemCommand(
			Commands.parallel(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				algaeIntakeStateHandler.handleIdle(),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.CLOSE)
			),
			"CLOSE"
		);
	}

	public Command removeAlgae() {
		return asSubsystemCommand(
			Commands.defer(
				() -> Commands.parallel(
					elevatorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getElevatorState()),
					armStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getArmState()),
					endEffectorStateHandler.setState(EndEffectorState.ALGAE_INTAKE_FROM_REEF),
					climbStateHandler.setState(ClimbState.STOP),
					algaeIntakeStateHandler.handleIdle()
				),
				subsystems
			),
			"REMOVE ALGAE"
		);
	}

	public Command intakeAlgae() {
		return asSubsystemCommand(
			Commands.parallel(
				Commands.sequence(
					algaeIntakeStateHandler.setState(AlgaeIntakeState.INTAKE).until(this::isAlgaeInIntake),
					algaeIntakeStateHandler.setState(AlgaeIntakeState.HOLD_ALGAE)
				),
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.STOP)
			),
			"INTAKE ALGAE"
		);
	}

	public Command holdAlgaeUp() {
		return asSubsystemCommand(
			Commands.parallel(
				elevatorStateHandler.setState(ElevatorState.HOLD_ALGAE),
				armStateHandler.setState(ArmState.HOLD_ALGAE),
				algaeIntakeStateHandler.handleIdle(),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT),
				climbStateHandler.setState(ClimbState.CLOSE)
			),
			"HOLD ALGAE UP"
		);
	}

    public Command net() {
        return asSubsystemCommand(
            Commands.parallel(
                Commands.sequence(
                    endEffectorStateHandler.setState(EndEffectorState.DEFAULT).until(net),
                    endEffectorStateHandler.setState(EndEffectorState.NET_OUTTAKE)
                ),
                elevatorStateHandler.setState(ElevatorState.NET),
                armStateHandler.setState(ArmState.NET),
                climbStateHandler.setState(ClimbState.STOP),
                algaeIntakeStateHandler.handleIdle()
            ),
            "NET"
        );
    }

	public Command intakeCoral() {
		return asSubsystemCommand(
			Commands.parallel(
				Commands.sequence(
					endEffectorStateHandler.setState(EndEffectorState.CORAL_INTAKE).until(this::isCoralIn),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				),
                elevatorStateHandler.setState(ElevatorState.INTAKE),
                armStateHandler.setState(ArmState.INTAKE),
                climbStateHandler.setState(ClimbState.STOP),
                algaeIntakeStateHandler.handleIdle()
			),
			"INTAKE CORAL"
		);
	}

}
