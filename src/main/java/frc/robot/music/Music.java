package frc.robot.music;

import com.ctre.phoenix6.Orchestra;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class Music extends GBSubsystem {

    private final Orchestra orchestra;
    private ArrayList<TalonFXMotor> instruments;
    private ArrayList<String> tracks;

    public Music(String logPath){
        super(logPath);
        this.orchestra = new Orchestra();
        instruments = new ArrayList<>();
    }

    @Override
    protected void subsystemPeriodic() {
        log();
    }

    private void log(){
        logInstruments();
        Logger.recordOutput(getLogPath() + "/isPlaying", orchestra.isPlaying());
        Logger.recordOutput(getLogPath() + "/currentTime", orchestra.getCurrentTime());
    }

    private void logInstruments(){
        for (TalonFXMotor motor : instruments){
            Logger.recordOutput(getLogPath() + "/Instruments/", motor.getLogPath());
        }
    }

    public void addMotors(TalonFXMotor... motors){
        for (TalonFXMotor motor : motors){
            orchestra.addInstrument(motor.getDevice());
            instruments.add(motor);
        }
    }

    public void addTrack(String filePath){
        tracks.add(filePath);
    }

    public void loadTrack(int trackNumber){
        orchestra.loadMusic(tracks.get(trackNumber));
    }

    public void play(){
        orchestra.play();
    }

    public void addSwerve(Swerve swerve){
        addMotors(
                swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getDrive(),
                swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getSteer(),

                swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_RIGHT).getDrive(),
                swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_RIGHT).getSteer(),

                swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_LEFT).getDrive(),
                swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_LEFT).getSteer(),

                swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_RIGHT).getDrive(),
                swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_RIGHT).getSteer()
        );
    }

}
