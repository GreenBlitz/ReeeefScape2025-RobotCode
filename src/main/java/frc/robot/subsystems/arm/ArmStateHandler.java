package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmStateHandler {

    private final Arm arm;

    public ArmStateHandler(Arm arm) {
        this.arm = arm;
    }

    public Command setState(ArmState state) {
        if (state == ArmState.STAY_IN_PLACE){
            return arm.getCommandsBuilder().stayInPlace();
        }
        return arm.getCommandsBuilder().moveToPosition(state.getPosition());
    }

    public Command endSate(ArmState state) {
        return switch (state) {
            case PRE_SCORE, INTAKE, STAY_IN_PLACE -> setState(ArmState.STAY_IN_PLACE);
            case L1, L2, L3, L4 -> setState(ArmState.PRE_SCORE)
        };
    }

}
