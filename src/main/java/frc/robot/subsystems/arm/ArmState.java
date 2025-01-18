package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {

    INTAKE(Rotation2d.fromDegrees(0)),
    L1(Rotation2d.fromDegrees(-1)),
    L2(Rotation2d.fromDegrees(-2)),
    L3(Rotation2d.fromDegrees(-3)),
    L4(Rotation2d.fromDegrees(-4)),
    PRE_SCORE(Rotation2d.fromDegrees(-5)),
    STAY_IN_PLACE(null);

    private final Rotation2d position;

    ArmState(Rotation2d position) {
        this.position = position;
    }

    public Rotation2d getPosition() {
        return position;
    }

}
