package frc.joysticks;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.JoystickBindings;
import frc.JoystickBindings.JoystickBindSet;

public class JoystickManager {

	private static final int NUMBER_OF_JOYSTICKS = 6;
	private static final SmartJoystick[] joysticks = new SmartJoystick[NUMBER_OF_JOYSTICKS];

	public static void setJoystickBindSet(int port, JoystickBindSet joystickBindSet, Robot robot) {
		if (joysticks[port] != null) {
			joysticks[port].setBindSet(joystickBindSet);
		} else if (joysticks[port] == null) {
			joysticks[port] = new SmartJoystick(port, joystickBindSet);
			JoystickBindings.configureBindings(joysticks[port], robot);
		}
	}

	private static void addOptions(SendableChooser<JoystickBindSet> chooser, int port, Robot robot) {
		chooser.setDefaultOption("NONE", JoystickBindSet.NONE);
		for (JoystickBindSet option : JoystickBindSet.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange((joystickBindSet) -> setJoystickBindSet(port, (chooser.getSelected()), robot));
		SmartDashboard.putData(port + " joystick", chooser);
	}

	public static void putChoosersToDashboard(Robot robot) {
		int currentPort = 0;
		for (SmartJoystick joystick : joysticks) {
			SendableChooser<JoystickBindSet> bindSetChooser = new SendableChooser<>();
			addOptions(bindSetChooser, currentPort, robot);
			currentPort++;
		}
	}

}
