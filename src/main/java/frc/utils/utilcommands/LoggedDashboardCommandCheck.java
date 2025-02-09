package frc.utils.utilcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class LoggedDashboardCommandCheck extends FunctionalCommand {
	LoggedDashboardNumber loggedDashboardNumber = new LoggedDashboardNumber("MotorVoltage",0);
	public LoggedDashboardCommandCheck(Runnable onInit, Runnable onExecute, Consumer<Boolean> onEnd, BooleanSupplier isFinished, Subsystem... requirements) {
		super(onInit, onExecute, onEnd, isFinished, requirements);
	}
	
	public LoggedDashboardCommandCheck(Consumer<Double> onExecute, String widgetName, Subsystem... requirements){
		super(
				()->{SmartDashboard.putString("Test Ran", "AHHHHHHHHHHHHHHHHHH");},
				()->{onExecute.accept(SmartDashboard.getNumber(widgetName,0)); SmartDashboard.putNumber("Test2",SmartDashboard.getNumber(widgetName,0));} ,
				(interrupted)->{},
				()->false,
				requirements
		);
		
	}
}
