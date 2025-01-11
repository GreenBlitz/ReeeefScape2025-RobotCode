package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.joysticks.SmartJoystick;

public class StatesMotionPlanner {

	private final Superstructure superstructure;

	public StatesMotionPlanner(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	public Command intakeTransfertoarmAmp(SmartJoystick joystick) {
		return superstructure.setState(RobotState.INTAKE, joystick)
			.andThen(superstructure.setState(RobotState.TRANSFER_SHOOTER_TO_ARM, joystick))
			.andThen(superstructure.setState(RobotState.AMP, joystick));
	}

}
