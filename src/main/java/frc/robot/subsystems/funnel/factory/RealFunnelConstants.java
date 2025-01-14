package frc.robot.subsystems.funnel.factory;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.FunnelStuff;

public class RealFunnelConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.1;

	private final static int LEFT_DIGITAL_INPUT_CHANNEL = 8;
	private final static int RIGHT_DIGITAL_INPUT_CHANNEL = 7;

	private final static LimitSwitchConfig.Type REVERSE_LIMIT_SWITCH_TYPE = LimitSwitchConfig.Type.kNormallyOpen;

	public static void configMotor(SparkMaxWrapper motor) {
		motor.setInverted(true);

		SparkMaxConfig config = new SparkMaxConfig();
		config.smartCurrentLimit(30);
		config.idleMode(SparkBaseConfig.IdleMode.kCoast);

		EncoderConfig encoderConfig = new EncoderConfig();
		encoderConfig.positionConversionFactor(FunnelConstants.GEAR_RATIO);
		encoderConfig.velocityConversionFactor(FunnelConstants.GEAR_RATIO);
		config.apply(encoderConfig);

		LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
		limitSwitchConfig.reverseLimitSwitchType(REVERSE_LIMIT_SWITCH_TYPE);
		config.apply(limitSwitchConfig);

		motor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
	}

	public static FunnelStuff generateFunnelStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXs.FUNNEL);
		configMotor(sparkMaxWrapper);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		sparkMaxWrapper.getReverseLimitSwitch().isPressed();
		ChanneledDigitalInput leftDigitalInput = new ChanneledDigitalInput(
			new DigitalInput(LEFT_DIGITAL_INPUT_CHANNEL),
			new Debouncer(DEBOUNCE_TIME_SECONDS)
		);
		ChanneledDigitalInput rightDigitalInput = new ChanneledDigitalInput(
			new DigitalInput(RIGHT_DIGITAL_INPUT_CHANNEL),
			new Debouncer(DEBOUNCE_TIME_SECONDS)
		);

		return new FunnelStuff(logPath, motor, voltageSignal, leftDigitalInput, rightDigitalInput);
	}

}
