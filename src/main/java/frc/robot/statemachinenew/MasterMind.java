package frc.robot.statemachinenew;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;

import java.util.function.BooleanSupplier;

public class MasterMind {


    // reef bind
    // net bind

    private boolean isAlgaeWantedUp = true;

    private boolean dontAutoReleaseReef = false;
    private boolean manualReleaseReef = true;


    private final Targets targets;
    private final Superstructure superstructure;
    private final SwerveControl swerveControl;

    public MasterMind(Robot robot) {
        this.targets = new Targets(robot);
        this.superstructure = new Superstructure("superstructure", robot, targets);
        this.swerveControl = new SwerveControl(superstructure, robot);

        new Trigger(() -> superstructure.isAlgaeInIntake() && isAlgaeWantedUp).onTrue(intake to hold);

        superstructure.setReefScoreTrigger(() ->  manualReleaseReef || targets.isReadyToScoreReef() && !dontAutoReleaseReef);
    }

    public Command scoreReef() {
        Command superstructureCommand = superstructure.scoreReef();
        if (ScoringHelpers.targetScoreLevel == ScoreLevel.L4) {
            superstructureCommand.beforeStarting(superstructure.whileDrive().until(targets::isReadyToOpenSuperstructure));
        }
        return superstructureCommand;
    }

}
