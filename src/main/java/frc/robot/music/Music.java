package frc.robot.music;

import com.ctre.phoenix6.Orchestra;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;

public class Music {

    private final Orchestra orchestra;

    public Music(){
        this.orchestra = new Orchestra();
    }

    public void addMotor(TalonFXMotor motor){
        orchestra.addInstrument(motor.getDevice());
    }

}
