package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class ElevatorCommandsBuilder {

	private final Elevator elevator;

	public ElevatorCommandsBuilder(Elevator elevator) {
		this.elevator = elevator;
	}

	public Command setPower(double power) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(power)), "Set power to " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(powerSupplier.getAsDouble())), "Set power by Supplier");
	}

	public Command setVoltage(double voltage) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setVoltage(voltage)), "Set voltage to " + voltage);
	}

	public Command setTargetPositionMeters(double targetPositionMeters) {
		return elevator.asSubsystemCommand(
			new RunCommand(() -> elevator.setTargetPositionMeters(targetPositionMeters)),
			"Set Target Position To " + targetPositionMeters + " Meters"
		);
	}

	public Command stayInPlace() {
		return elevator.asSubsystemCommand(new RunCommand(elevator::stayInPlace), "Stay in place");
	}

	public Command stop() {
		return elevator.asSubsystemCommand(new RunCommand(elevator::stop), "Stop");
	}

}