// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.factory.EndEffectorFactory;
import frc.utils.battery.BatteryUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final EndEffector endEffector;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		this.endEffector = EndEffectorFactory.create();
	}

	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public EndEffector getEndEffector() {
		return endEffector;
	}

}
