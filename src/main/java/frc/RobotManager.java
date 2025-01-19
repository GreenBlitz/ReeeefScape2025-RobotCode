// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSimulationHelper;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.time.TimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private Command autonomousCommand;
	private int roborioCycles;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
//		talonFXMotor.applyRequest(Phoenix6RequestBuilder.build(new VoltageOut(10)));
	}

	@Override
	public void testInit() {
//		talonFXMotor.applyRequest(Phoenix6RequestBuilder.build(new VoltageOut(-6)));
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		robot.periodic();
		AlertManager.reportAlerts();
		
		Logger.recordOutput("robot", new Pose3d(0,0,0,new Rotation3d()));
		Logger.recordOutput("elevator_stage_1", ElevatorSimulationHelper.getFirstStagePoseFromHeight(robot.elevator.getElevatorPositionMeters()));
		Logger.recordOutput("elevator_stage_2", ElevatorSimulationHelper.getSecondStagePoseFromHeight(robot.elevator.getElevatorPositionMeters()));
		Logger.recordOutput("arm", new Pose3d());

//		motor.updateSimulation();
//		motor.updateInputs(position, motorVol, supplyVol);
//		Logger.recordOutput("Tester/posMeters", Conversions.angleToDistance(positionSignla.getLatestValue(), 0.1));
//		talonFXMotor.updateSimulation();
//		talonFXMotor.updateInputs(voltageSignla, positionSignla);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtils.updateCycleTime(roborioCycles);
	}

}
