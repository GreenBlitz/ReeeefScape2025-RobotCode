package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;

public class SingingRobot {

    private final Orchestra orchestra;

    public SingingRobot(){
        this.orchestra = new Orchestra();
    }

    public void addMotor(TalonFXMotor motor){
        orchestra.addInstrument(motor.getDevice());
    }

}
