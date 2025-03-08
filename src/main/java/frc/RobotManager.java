// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.autonomous.AutosBuilder;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.StateMachineConstants;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtil;
import frc.utils.time.TimeUtil;
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
	private Command auto;
	private int roborioCycles;

	private final PathPlannerPath path;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

		this.path = AutosBuilder.getAutoScorePath(Branch.F, robot);
 		auto =
//		robot.getRobotCommander().asSubsystemCommand(
//			new DeferredCommand(
//				() ->
				new ParallelDeadlineGroup(
						new SequentialCommandGroup(
								robot.getRobotCommander().getSuperstructure().elevatorOpening(),
								robot.getRobotCommander().getSuperstructure().armPreScore().until(robot.getRobotCommander()::isReadyToOpenSuperstructure),
								robot.getRobotCommander().getSuperstructure().preScore().until(robot.getRobotCommander().getSuperstructure()::isPreScoreReady),
								robot.getRobotCommander().getSuperstructure().scoreWithoutRelease().until(robot.getRobotCommander()::isReadyToScore),
								robot.getRobotCommander().getSuperstructure().scoreWithRelease()
						),
						robot.getSwerve().asSubsystemCommand(PathFollowingCommandsBuilder.followPath(path)
								.andThen(
										robot.getSwerve().getCommandsBuilder()
												.moveToPoseByPID(
														() -> robot.getPoseEstimator().getEstimatedPose(),
														ScoringHelpers.getRobotBranchScoringPose(
																ScoringHelpers.getTargetBranch(),
																StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
														)
												)
								), "NAMENAME"
						)
				);
//				Set.of(
//					this,
//					superstructure,
//					swerve,
//					robot.getElevator(),
//					robot.getArm(),
//					robot.getEndEffector(),
//					robot.getLifter(),
//					robot.getSolenoid()
//				)
//			),
//				RobotState.SCORE.name()
//		);
		createAutoReadyForConstructionChooser();
		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		BrakeStateManager.brake();
	}



	@Override
	public void autonomousInit() {

//		if (auto == null) {
//			this.auto = robot.getAuto();
//		}
		auto.schedule();
	}

	@Override
	public void teleopInit() {
		if (auto != null) {
			auto.cancel();
		}
		robot.getRobotCommander().initializeDefaultCommand();
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.setDriversInputsToSwerve(robot.getSwerve());
		robot.periodic();
		AlertManager.reportAlerts();
	}

	private void createAutoReadyForConstructionChooser() {
		SendableChooser<Boolean> autoReadyForConstructionSendableChooser = new SendableChooser<>();
		autoReadyForConstructionSendableChooser.setDefaultOption("false", false);
		autoReadyForConstructionSendableChooser.addOption("true", true);
		autoReadyForConstructionSendableChooser.onChange(isReady -> {
			if (isReady) {
//				auto = robot.getAuto();
			}
		});
		SmartDashboard.putData("AutoReadyForConstruction", autoReadyForConstructionSendableChooser);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}
