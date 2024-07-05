// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimation.poseestimator.PoseCalculator;
import frc.robot.poseestimation.poseestimator.PoseEstimationSuperstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveState;
import frc.utils.DriverStationUtils;
import frc.utils.RobotTypeUtils;
import frc.utils.pathplannerutils.PathPlannerUtils;


public class Robot {

    public static final RobotTypeUtils.RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.REAL);

    private static final PoseCalculator poseCalculator = new PoseCalculator();

    public static final Swerve swerve = new Swerve();

    public static final PoseEstimationSuperstructure poseEstimator = new PoseEstimationSuperstructure(poseCalculator, swerve);

    public Robot() {
        buildPathPlannerForAuto();
        configureBindings();
    }

    private void buildPathPlannerForAuto() {
        // Register commands...
        PathPlannerUtils.configurePathPlanner(
                Robot.poseEstimator::getCurrentPose,
                Robot.poseEstimator::resetPose,
                Robot.swerve::getSelfRelativeVelocity,
                (speeds) -> Robot.swerve.driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER), // todo: use command
                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                DriverStationUtils::isRedAlliance,
                Robot.swerve
        );
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}
