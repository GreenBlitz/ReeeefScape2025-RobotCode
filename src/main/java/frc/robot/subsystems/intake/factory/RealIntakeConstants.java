package frc.robot.subsystems.intake.factory;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeStuff;

import java.util.function.BooleanSupplier;

public class RealIntakeConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static LimitSwitchConfig.Type REVERSE_LIMIT_SWITCH_TYPE = LimitSwitchConfig.Type.kNormallyOpen;

	private static void configMotor(SparkMaxWrapper motor) {
		motor.setInverted(false);

		SparkMaxConfig config = new SparkMaxConfig();
		config.smartCurrentLimit(30);
		config.idleMode(SparkBaseConfig.IdleMode.kCoast);

		EncoderConfig encoderConfig = new EncoderConfig();
		encoderConfig.positionConversionFactor(IntakeConstants.GEAR_RATIO);
		encoderConfig.velocityConversionFactor(IntakeConstants.GEAR_RATIO);
		config.apply(encoderConfig);

		LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
		limitSwitchConfig.reverseLimitSwitchType(REVERSE_LIMIT_SWITCH_TYPE);
		limitSwitchConfig.reverseLimitSwitchEnabled(false);
		config.apply(limitSwitchConfig);

		motor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
	}

	public static IntakeStuff generateIntakeStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXs.INTAKE);
		configMotor(sparkMaxWrapper);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		BooleanSupplier isPressed = () -> sparkMaxWrapper.getReverseLimitSwitch().isPressed();
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isPressed, new Debouncer(DEBOUNCE_TIME_SECONDS, DEBOUNCE_TYPE));

		return new IntakeStuff(logPath, motor, voltageSignal, beamBreaker);
	}

}
