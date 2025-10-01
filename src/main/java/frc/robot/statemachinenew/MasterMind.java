package frc.robot.statemachinenew;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;

import java.util.function.BooleanSupplier;

public class MasterMind {


    // reef bind
    // net bind

    private boolean dontAutoReleaseNet = true;
    private boolean manualReleaseNet = true;

    private boolean isAlgaeWantedUp = true;

    private boolean dontAutoReleaseReef = false;
    private boolean manualReleaseReef = true;


    private final Targets targets;
    private final Superstructure superstructure;
    private final SwerveControl swerveControl;

    public MasterMind(Robot robot) {
        this.targets = new Targets(robot);
        this.superstructure = new Superstructure("superstrcuture", robot, targets);
        this.swerveControl = new SwerveControl(superstructure, robot);

        superstructure.setScoreReef(() ->  manualReleaseReef || targets.isReadyToScoreReef() && !dontAutoReleaseReef);
    }

    public Command scoreReef() {
        if (ScoringHelpers.targetScoreLevel == ScoreLevel.L4) {
            Commands.sequence(
                superstructure.whileDrive().until(() -> targets.isReadyToOpenSuperstructure()),
                superstructure.scoreReef()
            );
        }
    }

}
