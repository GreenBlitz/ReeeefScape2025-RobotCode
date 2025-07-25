// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.led.LEDConstants;
import frc.robot.led.LEDState;
import frc.robot.subsystems.climb.lifter.LifterConstants;
import frc.utils.DriverStationUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.time.TimeUtil;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {
	
	SmartJoystick joystick = new SmartJoystick(JoystickPorts.MAIN);
	DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.4);
	TalonSRX[] rightMotors = new TalonSRX[]{
			new TalonSRX(1),
			new TalonSRX(2)
	};
	TalonSRX[] leftMotors = new TalonSRX[]{
			new TalonSRX(3),
			new TalonSRX(4)
	};
	
	public RobotManager() {
		LoggerFactory.initializeLogger();
		DriverStation.silenceJoystickConnectionWarning(true);
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();
		
		Arrays.stream(leftMotors).forEach((motor) -> motor.setInverted(true));
	}
	
	@Override
	public void robotPeriodic() {
		AlertManager.reportAlerts();
		
		DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(
				new ChassisSpeeds(
						joystick.getAxisValue(Axis.LEFT_Y) * 0.8,
						joystick.getAxisValue(Axis.LEFT_Y) * 0.8,
						joystick.getAxisValue(Axis.RIGHT_X) * -4)
		);
		Arrays.stream(rightMotors).forEach((motor) -> motor.set(TalonSRXControlMode.PercentOutput, speeds.rightMetersPerSecond * 0.5));
		Arrays.stream(leftMotors).forEach((motor) -> motor.set(TalonSRXControlMode.PercentOutput, speeds.leftMetersPerSecond * 0.5));
		
	}
}
