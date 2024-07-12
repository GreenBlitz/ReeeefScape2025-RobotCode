package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

    @Override
    public final void periodic() {
        Logger.recordOutput("Subsystems/"+ getLogPath() + "Current Command", getCurrentCommandName());
        subsystemPeriodic();
    }

    private String getCurrentCommandName() {
        Command currentCommand = getCurrentCommand();
        return currentCommand != null ? currentCommand.getName() : "no command is currently running on the subsystem";
    }

    public String getLogPath(){
        return "Subsystems/"+getName()+"/";
    }

    protected abstract void subsystemPeriodic();

}
