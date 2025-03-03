package frc.robot.subsystems.climb.solenoid;

import edu.wpi.first.wpilibj2.command.Command;

public class SolenoidStateHandler {

	private final Solenoid solenoid;

	public SolenoidStateHandler(Solenoid solenoid) {
		this.solenoid = solenoid;
	}

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Command setState(SolenoidState state) {
		return solenoid.getCommandsBuilder().setPower(state.getPower());
	}

}
