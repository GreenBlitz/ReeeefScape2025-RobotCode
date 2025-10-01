package frc.robot.statemachinenew;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.scoringhelpers.ScoringPathsHelper;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Set;

public class SwerveControl {

    // known issues:
    // autmations
    // need to add a star
    // default command

    public boolean assistNet = true;
    public boolean assistReef = true;

    private final Swerve swerve;
    private final IPoseEstimator poseEstimator;

    public SwerveControl(Superstructure superstructure, Robot robot) {
        this.swerve = robot.getSwerve();
        this.poseEstimator = robot.getPoseEstimator();

        new Trigger(() -> superstructure.getCurrentState().taskIs(State.Task.REEF) && assistReef).whileTrue(reefAssist());
        new Trigger(() -> superstructure.getCurrentState().taskIs(State.Task.NET) && assistNet).whileTrue(netAssist());
    }

    public Command reefAssist() {
        return Commands.defer(
            () -> swerve.getCommandsBuilder().driveToPath(
                poseEstimator::getEstimatedPose,
                ScoringPathsHelper.getPathByBranch(ScoringHelpers.getTargetBranch()),
                AutonomousConstants.getRealTimeConstraints(swerve)
            ), Set.of(swerve)
        );
    }

    public Command netAssist() {
        return new InstantCommand();
    }

}
