package frc.robot.statemachinenew;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

import static frc.robot.statemachinenew.State.NET;
import static frc.robot.statemachinenew.State.SCORE_REEF;

public class SwerveControl {

    // known issues:
    // swerve control
    // autmations
    // need to add a star
    // default command

    private boolean aaNet = true;
    private boolean aaReef = true;
    private boolean aaProcessor = true;

    public SwerveControl(Superstructure superstructure, Robot robot) {
        Trigger t = new Trigger(superstructure.getState == NET && aaNet).whileTrue(net assist);
        Trigger t = new Trigger(superstructure.getState == SCORE_REEF && aaReef).whileTrue(reef assist);
    }

}
