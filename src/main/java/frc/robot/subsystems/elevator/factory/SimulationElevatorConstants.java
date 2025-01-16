package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.ElevatorSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.records.ElevatorRequests;
import frc.robot.subsystems.elevator.records.ElevatorSignals;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class SimulationElevatorConstants {

    private static final double LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;
    private static final double RADIUS_METERS = 0.025;
    private static final int NUMBER_OF_MOTORS = 2;
    private static final double MIN_HEIGHT_METERS = 0;
    private static final double MAX_HEIGHT_METERS = 2;
    private static final double STARTING_HEIGHT_METERS = 0;
    private static final double GEAR_RATIO = 1.0 / 5.0;
    private static final double CURRENT_LIMIT = 40;
    private static final boolean CURRENT_LIMIT_ENABLE = true;
    private static final boolean SOFT_LIMIT_ENABLE = true;
    private static final SysIdRoutine.Config MOTOR_CONFIG = new SysIdRoutine.Config();

    private static final double KP = 1;
    private static final double KI = 0;
    private static final double KD = 0;

    private static void configMotor(TalonFXMotor motor){
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Slot0.withKP(KP).withKI(KI).withKD(KD);
        configuration.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        configuration.CurrentLimits.StatorCurrentLimitEnable = CURRENT_LIMIT_ENABLE;
        configuration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(Elevator.convertMetersToRotations(MIN_HEIGHT_METERS).getRotations());
        configuration.SoftwareLimitSwitch.withReverseSoftLimitEnable(SOFT_LIMIT_ENABLE);
        configuration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Elevator.convertMetersToRotations(MAX_HEIGHT_METERS).getRotations());
        configuration.SoftwareLimitSwitch.withForwardSoftLimitEnable(SOFT_LIMIT_ENABLE);
        motor.applyConfiguration(configuration);
    }

//    private static ElevatorRequests createRequests(){
//        return
//    }

//    private static ElevatorSignals createSignals(TalonFXMotor motor){
//        return
//    }

    public static Elevator generate(String logPath){
        ElevatorSimulation elevatorSimulation = new ElevatorSimulation(
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                DCMotor.getKrakenX60Foc(2),
                                ElevatorConstants.MASS_KG,
                                RADIUS_METERS,
                                ElevatorConstants.DRUM_RADIUS
                        ),
                        DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
                        MIN_HEIGHT_METERS,
                        MAX_HEIGHT_METERS,
                        false,
                        STARTING_HEIGHT_METERS
                ),
                ElevatorConstants.DRUM_RADIUS,
                GEAR_RATIO
        );
        elevatorSimulation.setInputVoltage(12);
        TalonFXMotor firstMotor = new TalonFXMotor(logPath + "FirstMotor/", IDs.Phoenix6IDs.ELEVATOR_FIRST_MOTOR_ID, MOTOR_CONFIG, elevatorSimulation);
        configMotor(firstMotor);

        SuppliedDigitalInput digitalInput = new SuppliedDigitalInput(() -> false, new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME));

        ElevatorSignals signals = new ElevatorSignals(
                Phoenix6SignalBuilder.generatePhoenix6Signal(firstMotor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(firstMotor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ)
        );

        ElevatorRequests requests = new ElevatorRequests(
                Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(true)),
                Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true))
        );

        Logger.recordOutput("pos", firstMotor.getDevice().getPosition().getValueAsDouble());
        return new Elevator(
                logPath,
                logPath + "LimitSwitch/",
                firstMotor,
                signals,
                requests,
                firstMotor,
                signals,
                requests,
                digitalInput
        );
    }

}