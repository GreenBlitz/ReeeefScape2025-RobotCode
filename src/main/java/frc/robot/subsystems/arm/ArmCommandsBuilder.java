package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

public class ArmCommandsBuilder {

    private final Arm arm;

    public ArmCommandsBuilder(Arm arm) {
        this.arm = arm;
    }

    public Command moveToPosition(Rotation2d position) {
        return arm.asSubsystemCommand(new InstantCommand(() -> arm.setTargetPosition(position)), "Move arm to position: " + position);
    }

    public Command stayInPlace() {
        return arm.asSubsystemCommand(new RunCommand(arm::stayInPlace), "Arm stay in place");
    }

    public Command setVoltage(double voltage) {
        return arm.asSubsystemCommand(new RunCommand(() -> arm.setVoltage(voltage)), "Set arm voltage to: " + voltage);
    }

    public Command setPower(double power) {
        return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(power)), "Set arm power to: " + power);
    }

    public Command loggedDashboardSetVoltage() {
        return arm.asSubsystemCommand(
                new LoggedDashboardCommand("Set Example Arm Voltage", (voltage) -> arm.setVoltage(voltage)),
                "Set arm voltage through dashboard, set voltage to the given value"
        );
    }

}
