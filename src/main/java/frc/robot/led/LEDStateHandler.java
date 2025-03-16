package frc.robot.led;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;

public class LEDStateHandler extends GBSubsystem {

	private final CANdle candle;

	public LEDStateHandler(String logPath, CANdle candle) {
		super(logPath);
		this.candle = candle;

		setDefaultCommand(new ConditionalCommand(setState(LEDState.IDLE), setState(LEDState.DISABLE), DriverStation::isEnabled));
	}

	public Command setState(LEDState state) {
		Command setStateCommand = asSubsystemCommand(
				new ParallelCommandGroup(
						new RunCommand(() -> candle.animate(state.getAnimation()), this)
								.ignoringDisable(true)
				), state.name());

		if (state == LEDState.HAS_CORAL) {
			return setStateCommand.withTimeout(LEDConstants.TIME_FOR_HAS_CORAL_STATE).ignoringDisable(true);
		}

		return setStateCommand;
	}

}
