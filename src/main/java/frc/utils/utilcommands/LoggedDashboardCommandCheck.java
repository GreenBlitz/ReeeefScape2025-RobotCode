package frc.utils.utilcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class LoggedDashboardCommandCheck extends FunctionalCommand {
	LoggedDashboardNumber loggedDashboardNumber = new LoggedDashboardNumber("MotorVoltage",0);
	public LoggedDashboardCommandCheck(Runnable onInit, Runnable onExecute, Consumer<Boolean> onEnd, BooleanSupplier isFinished, Subsystem... requirements) {
		super(onInit, onExecute, onEnd, isFinished, requirements);
	}
	
	public  LoggedDashboardCommandCheck(Consumer<Double> onExecute, String widgetName, Subsystem... requirements){
		super(
				()->{SmartDashboard.updateValues();
					SmartDashboard.putString("Test Ran", "AHHHHHHHHHHHHHHHHHH");},
				()->{onExecute.accept(new LoggedTunableNumber("SmartDashboard/VoltageSomething",4).get());
					SmartDashboard.putNumber("Test2",new LoggedTunableNumber("SmartDashboard/VoltageSomething",4).get());} ,
				(interrupted)->{},
				()->false,
				requirements
		);
		
	}
}
