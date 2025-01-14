package frc.robot.subsystems.roller.factory;

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
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealRollerConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.1;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static LimitSwitchConfig.Type REVERSE_LIMIT_SWITCH_TYPE = LimitSwitchConfig.Type.kNormallyOpen;

	private static void configMotor(SparkMaxWrapper motor) {
		motor.setInverted(false);
		SparkMaxConfig config = new SparkMaxConfig();
		config.smartCurrentLimit(30);
		config.idleMode(SparkBaseConfig.IdleMode.kCoast);

		EncoderConfig encoderConfig = new EncoderConfig();
		encoderConfig.positionConversionFactor(RollerConstants.GEAR_RATIO);
		encoderConfig.velocityConversionFactor(RollerConstants.GEAR_RATIO);
		config.apply(encoderConfig);

		LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
		limitSwitchConfig.reverseLimitSwitchType(REVERSE_LIMIT_SWITCH_TYPE);
		limitSwitchConfig.reverseLimitSwitchEnabled(false);
		config.apply(limitSwitchConfig);

		motor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
	}

	public static RollerStuff generateRollerStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXs.ROLLER);
		configMotor(sparkMaxWrapper);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		Supplier<Double> positionSupplier = () -> sparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", positionSupplier, AngleUnit.ROTATIONS);

		BooleanSupplier isBeamBroken = () -> sparkMaxWrapper.getReverseLimitSwitch().isPressed();
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isBeamBroken, new Debouncer(DEBOUNCE_TIME_SECONDS, DEBOUNCE_TYPE));

		return new RollerStuff(logPath, motor, voltageSignal, positionSignal, beamBreaker);
	}

}
