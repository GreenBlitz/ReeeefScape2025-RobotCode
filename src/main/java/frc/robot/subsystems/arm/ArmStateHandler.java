package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmStateHandler {

    private final Arm arm;

    public ArmStateHandler(Arm arm) {
        this.arm = arm;
    }

    public Command setState(ArmState state) {
        return switch (state) {
            case STAY_IN_PLACE -> arm.getCommandsBuilder().stayInPlace();
            case INTAKE, SAFE_HOLD, LOW_DROP, MID_DROP, HIGH_DROP -> arm.getCommandsBuilder().moveToPosition(state.getPosition());
        };
    }

    public Command endSate(ArmState state) {
        return switch (state) {
            case SAFE_HOLD, HIGH_DROP, MID_DROP, LOW_DROP, INTAKE, STAY_IN_PLACE -> setState(ArmState.STAY_IN_PLACE);
        };
    }

}
