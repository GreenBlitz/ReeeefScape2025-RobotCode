// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.mechanisms.wpilib.ElevatorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.Conversions;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.time.TimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private Command autonomousCommand;
	private int roborioCycles;

//	private final Elevator elevator;
//	private final TalonFXMotor motor;
//	private final Phoenix6AngleSignal position;
//	private final Phoenix6DoubleSignal motorVol;
//	private final Phoenix6DoubleSignal supplyVol;
//	private final ElevatorSimulation simulation;

	public static TalonFXMotor talonFXMotor = new TalonFXMotor(
			"Tester/",
			new Phoenix6DeviceID(1),
			new SysIdRoutine.Config(),
			new ElevatorSimulation(
					new ElevatorSim(
							LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60Foc(1), 4, 0.05, 10),
							DCMotor.getKrakenX60Foc(1),
							0,
							2.5,
							false,
							0
					),
					0.1,
					10
			)
	);
	static {
		talonFXMotor.applyConfiguration(new TalonFXConfiguration().withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(500).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(0)));
	}
	public static Phoenix6DoubleSignal voltageSignla = Phoenix6SignalBuilder
			.generatePhoenix6Signal(talonFXMotor.getDevice().getMotorVoltage(), 50);
	public static Phoenix6AngleSignal positionSignla = Phoenix6SignalBuilder
			.generatePhoenix6Signal(talonFXMotor.getDevice().getPosition(), 50, AngleUnit.ROTATIONS);


	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

//		elevator = ElevatorFactory.create(ElevatorConstants.LOG_PATH);
//		simulation = new ElevatorSimulation(
//				new ElevatorSim(
//						DCMotor.getKrakenX60Foc(1),
//						1.0/10.0,
//						5,
//						0.05,
//						0,
//						2,
//						false,
//						0
//				),
//				0.05,
//				1.0/10.0
//		);
//		motor = new TalonFXMotor("Test/", new Phoenix6DeviceID(1), new SysIdRoutine.Config(), simulation);
//
//		position = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getDevice().getPosition(), 60, AngleUnit.ROTATIONS);
//		motorVol = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getDevice().getMotorVoltage(), 60);
//		supplyVol = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getDevice().getSupplyVoltage(), 60);

		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
		talonFXMotor.applyRequest(Phoenix6RequestBuilder.build(new VoltageOut(12)));
	}
	@Override
	public void testInit() {
		talonFXMotor.applyRequest(Phoenix6RequestBuilder.build(new VoltageOut(-6)));
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
//		elevator.getCommandsBuilder().setVoltage(122).schedule();
//		motor.applyRequest(Phoenix6RequestBuilder.build(new VoltageOut(12)));
//		simulation.setInputVoltage(12);
		talonFXMotor.applyRequest(Phoenix6RequestBuilder.build(new VoltageOut(0)));

	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		robot.periodic();
		AlertManager.reportAlerts();

//		motor.updateSimulation();
//		motor.updateInputs(position, motorVol, supplyVol);
		Logger.recordOutput("Tester/posMeters", Conversions.angleToDistance(positionSignla.getLatestValue(), 0.1));
		talonFXMotor.updateSimulation();
		talonFXMotor.updateInputs(voltageSignla, positionSignla);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtils.updateCycleTime(roborioCycles);
	}

}