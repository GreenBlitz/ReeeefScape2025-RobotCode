package frc.robot.subsystems.elbow.factory;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.ElbowStuff;
import frc.utils.AngleUnit;

import java.util.function.Function;

public class RealElbowConstants {

	private static final double KS = 0.15;
	private static final double KG = 0.2;
	private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, 0);

	private static final ClosedLoopConfig.ClosedLoopSlot POSITION_PID_SLOT = ClosedLoopConfig.ClosedLoopSlot.kSlot0;

	private static SparkMaxRequest<Rotation2d> generatePositionRequest() {
		Function<Rotation2d, Double> feedforwardCalculation = setPoint -> FEEDFORWARD.calculate(setPoint.getRadians(), 0);
		return SparkMaxRequestBuilder.build(new Rotation2d(), SparkBase.ControlType.kPosition, POSITION_PID_SLOT.value, feedforwardCalculation);
	}

	private static SparkMaxRequest<Double> generateVoltageRequest() {
		return SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kVoltage, 0);
	}

	private static void configMotor(SparkMaxWrapper motor) {
		motor.setInverted(true);

		SparkMaxConfig config = new SparkMaxConfig();
		config.smartCurrentLimit(40);
		config.idleMode(SparkBaseConfig.IdleMode.kBrake);

		EncoderConfig encoderConfig = new EncoderConfig();
		encoderConfig.positionConversionFactor(ElbowConstants.GEAR_RATIO);
		encoderConfig.velocityConversionFactor(ElbowConstants.GEAR_RATIO);
		config.apply(encoderConfig);

		SoftLimitConfig softLimitConfig = new SoftLimitConfig();
		softLimitConfig.forwardSoftLimit(ElbowConstants.FORWARD_LIMIT.getRotations());
		softLimitConfig.forwardSoftLimitEnabled(true);
		softLimitConfig.reverseSoftLimit(ElbowConstants.BACKWARD_LIMIT.getRotations());
		softLimitConfig.reverseSoftLimitEnabled(true);
		config.apply(softLimitConfig);

		ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
		closedLoopConfig.p(5.5, POSITION_PID_SLOT);
		closedLoopConfig.d(0.5, POSITION_PID_SLOT);
		config.apply(closedLoopConfig);

		motor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
	}

	protected static ElbowStuff generateElbowStuff(String logPath) {
		SparkMaxWrapper motor = new SparkMaxWrapper(IDs.SparkMAXs.ELBOW);
		configMotor(motor);

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", () -> motor.getEncoder().getPosition(), AngleUnit.ROTATIONS);
		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", () -> motor.getEncoder().getVelocity(), AngleUnit.ROTATIONS);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("output current", motor::getOutputCurrent);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", motor::getVoltage);

		BrushlessSparkMAXMotor elbow = new BrushlessSparkMAXMotor(logPath, motor, new SysIdRoutine.Config());
		return new ElbowStuff(
			logPath,
			elbow,
			generatePositionRequest(),
			generateVoltageRequest(),
			positionSignal,
			velocitySignal,
			currentSignal,
			voltageSignal
		);
	}

}
