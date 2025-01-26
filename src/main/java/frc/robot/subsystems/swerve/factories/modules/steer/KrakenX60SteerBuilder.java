package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;
import frc.utils.math.AngleUnit;

class KrakenX60SteerBuilder {

	private static final double GEAR_RATIO = 12.8;

	private static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(
			Units.Volts.of(0.5).per(Units.Second),
			Units.Volts.of(1),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static SimpleMotorSimulation buildMechanismSimulation() {
		double momentOfInertiaMetersSquared = 1.0E-5;
		return new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), momentOfInertiaMetersSquared, GEAR_RATIO),
				DCMotor.getKrakenX60Foc(1)
			)
		);
	}

	private static TalonFXConfiguration buildMotorConfig(boolean inverted) {
		TalonFXConfiguration steerConfig = new TalonFXConfiguration();

		steerConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		steerConfig.CurrentLimits.StatorCurrentLimit = 30;
		steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		steerConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
		steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

		if (Robot.ROBOT_TYPE.isReal()) {
			steerConfig.Slot0.kS = 0;
			steerConfig.Slot0.kV = 0;
			steerConfig.Slot0.kA = 0;
			steerConfig.Slot0.kP = 0;
			steerConfig.Slot0.kI = 0;
			steerConfig.Slot0.kD = 0;
		} else {
			steerConfig.Slot0.kS = 0;
			steerConfig.Slot0.kV = 0;
			steerConfig.Slot0.kA = 0;
			steerConfig.Slot0.kP = 35;
			steerConfig.Slot0.kI = 0;
			steerConfig.Slot0.kD = 0;
		}
		steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

		return steerConfig;
	}

	static ControllableMotor buildSteer(String logPath, Phoenix6DeviceID deviceID, Phoenix6DeviceID encoderID, boolean inverted) {
		TalonFXConfiguration configuration = buildMotorConfig(inverted);
		configuration.Feedback.FeedbackRemoteSensorID = encoderID.id();

		TalonFXMotor steer = new TalonFXMotor(logPath, deviceID, buildSysidConfig(), buildMechanismSimulation());
		steer.applyConfiguration(configuration);
		return steer;
	}

	static SteerRequests buildRequests() {
		return new SteerRequests(
			Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(true)),
			Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true))
		);
	}

	static SteerSignals buildSignals(TalonFXMotor steer) {
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getPosition(), velocitySignal, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		return new SteerSignals(positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
