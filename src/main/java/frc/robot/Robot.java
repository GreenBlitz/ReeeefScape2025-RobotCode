// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSimulationHelper;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.utils.battery.BatteryUtils;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	public static final Mechanism2d mech2d = new Mechanism2d(20,20);

	public final Elevator elevator;
	
	public Robot() {
		BatteryUtils.scheduleLimiter();
		SmartDashboard.putData("Mechanism2d", mech2d);

		elevator = ElevatorFactory.create("Elevator/");
	}

	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run();// Should be last
		
		logElevatorSimulationPositions(elevator.getElevatorPositionMeters());
	}
	
	private void logElevatorSimulationPositions(double heightInMeters){
		Logger.recordOutput("robot", new Pose2d());
		Logger.recordOutput("arm", ElevatorSimulationHelper.getSecondStagePoseFromHeight(heightInMeters));
		Logger.recordOutput("elevator stage 1", ElevatorSimulationHelper.getFirstStagePoseFromHeight(heightInMeters));
		Logger.recordOutput("elevator stage 2", ElevatorSimulationHelper.getSecondStagePoseFromHeight(heightInMeters));
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}
