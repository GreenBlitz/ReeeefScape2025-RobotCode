// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.factory.FunnelFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.factory.IntakeFactory;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.factory.ElbowFactory;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.factory.FlywheelFactory;
import frc.robot.subsystems.lifter.Lifter;
import frc.robot.subsystems.lifter.LifterConstants;
import frc.robot.subsystems.lifter.factory.LifterFactory;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.factory.PivotFactory;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.factory.RollerFactory;
import frc.robot.subsystems.solenoid.Solenoid;
import frc.robot.subsystems.solenoid.SolenoidConstants;
import frc.robot.subsystems.solenoid.factory.SolenoidFactory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.constants.GlobalConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.factory.WristFactory;
import frc.robot.superstructure.StatesMotionPlanner;
import frc.robot.superstructure.Superstructure;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.auto.AutonomousChooser;

import frc.utils.battery.BatteryUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private AutonomousChooser autonomousChooser;


	private final Swerve swerve;
	private final Solenoid solenoid;
	private final Funnel funnel;
	private final Intake intake;
	private final Elbow elbow;
	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Lifter lifter;
	private final Roller roller;
	private final Wrist wrist;

	private final Superstructure superstructure;
	private final StatesMotionPlanner statesMotionPlanner;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(GlobalConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(GlobalConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			ModulesFactory.create(GlobalConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.solenoid = new Solenoid(SolenoidFactory.create(SolenoidConstants.LOG_PATH));
		this.intake = new Intake(IntakeFactory.create(IntakeConstants.LOG_PATH));
		this.flywheel = new Flywheel(FlywheelFactory.create(FlyWheelConstants.LOG_PATH));
		this.pivot = new Pivot(PivotFactory.create(PivotConstants.LOG_PATH));
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));
		this.elbow = new Elbow(ElbowFactory.create(ElbowConstants.LOG_PATH));
		BrakeStateManager.add(() -> elbow.setBrake(true), () -> elbow.setBrake(false));
		this.funnel = new Funnel(FunnelFactory.create(FunnelConstants.LOG_PATH));
		this.lifter = new Lifter(LifterFactory.create(LifterConstants.LOG_PATH));
		BrakeStateManager.add(() -> lifter.setBrake(true), () -> lifter.setBrake(false));
		this.roller = new Roller(RollerFactory.create(RollerConstants.LOG_PATH));
		BrakeStateManager.add(() -> roller.setBrake(true), () -> roller.setBrake(false));
		this.wrist = new Wrist(WristFactory.create(WristConstants.LOG_PATH));
		BrakeStateManager.add(() -> wrist.setBrake(true), () -> wrist.setBrake(false));

//		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
//		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		this.superstructure = new Superstructure("Superstructure/", this);
		this.statesMotionPlanner = new StatesMotionPlanner(superstructure);

		configPathPlanner();
	}

	public void periodic() {
		swerve.update();
//		poseEstimator.updateVision(limelightFilterer.getFilteredVisionObservations());
//		poseEstimator.updateOdometry(Arrays.asList(swerve.getAllOdometryObservations()));
		superstructure.logStatus();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	private void configPathPlanner() {
		// Register commands..
//		PathPlannerUtils.registerCommand(RobotState.INTAKE_WITH_FLYWHEEL.name(), superstructure.setState(RobotState.INTAKE_WITH_FLYWHEEL));
//		PathPlannerUtils.registerCommand(RobotState.PRE_SPEAKER.name(), superstructure.setState(RobotState.PRE_SPEAKER));
//		PathPlannerUtils.registerCommand(RobotState.SPEAKER.name(), superstructure.setState(RobotState.SPEAKER));

//		swerve.configPathPlanner(
//			poseEstimator::getEstimatedPose,
//			poseEstimator::resetPose,
//			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
//		);
		autonomousChooser = new AutonomousChooser("Autonomous Chooser");
	}


	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Funnel getFunnel() {
		return funnel;
	}

	public Intake getIntake() {
		return intake;
	}

	public Elbow getElbow() {
		return elbow;
	}

	public Flywheel getFlywheel() {
		return flywheel;
	}

	public Pivot getPivot() {
		return pivot;
	}

	public Lifter getLifter() {
		return lifter;
	}

	public Roller getRoller() {
		return roller;
	}

	public Wrist getWrist() {
		return wrist;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public StatesMotionPlanner getStatesMotionPlanner() {
		return statesMotionPlanner;
	}

}
