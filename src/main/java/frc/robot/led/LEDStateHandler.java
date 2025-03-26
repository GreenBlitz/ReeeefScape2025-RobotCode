package frc.robot.led;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;

public class LEDStateHandler extends GBSubsystem {

	private final CANdle candle;

	public LEDStateHandler(String logPath, CANdle candle) {
		super(logPath);
		this.candle = candle;

		setDefaultCommand(
			new ConditionalCommand(new DeferredCommand(this::getIdleByAlliance, Set.of(this)), setState(LEDState.DISABLE), DriverStation::isEnabled).ignoringDisable(true)
		);
	}

	public Command getIdleByAlliance(){
		if (DriverStation.getAlliance().isEmpty()){
			return setState(LEDState.DISABLE);
		}
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
			return setState(LEDState.IDLE_RED);
		}else{
			return setState(LEDState.IDLE_BLUE);
		}
	}

	public Command setState(LEDState state) {
		Command setStateCommand = asSubsystemCommand(
			new InitExecuteCommand(() -> candle.animate(state.getAnimation()), () -> {}, this)
				.withTimeout(LEDConstants.CORAL_IN_BLINK_TIME_SECONDS)
				.ignoringDisable(true),
			state.name()
		);

		if (state == LEDState.HAS_CORAL) {
			return setStateCommand.withTimeout(LEDConstants.TIME_FOR_HAS_CORAL_STATE).ignoringDisable(true);
		}

		return setStateCommand;
	}

}
