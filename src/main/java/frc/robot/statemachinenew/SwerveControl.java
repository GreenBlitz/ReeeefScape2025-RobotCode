package frc.robot.statemachinenew;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.scoringhelpers.ScoringPathsHelper;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Set;

import static frc.robot.statemachinenew.State.SCORE_REEF;

public class SwerveControl {

    // known issues:
    // autmations
    // need to add a star
    // default command

    public boolean aaNet = true;
    public boolean assistReef = true;
    public boolean aaProcessor = true;

    private final Swerve swerve;
    private final IPoseEstimator poseEstimator;

    public SwerveControl(Superstructure superstructure, Robot robot) {
        this.swerve = robot.getSwerve();
        this.poseEstimator = robot.getPoseEstimator();

        new Trigger(superstructure.getState == SCORE_REEF && assistReef).whileTrue(reefAssist());
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

}
