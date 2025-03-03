package frc.robot.subsystems.climb.solenoid.factory;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.IDs;
import frc.robot.RobotType;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.*;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.solenoid.Solenoid;

public class SparkMaxSolenoidBuilder {

	private static final int NUMBER_OF_MOTORS = 1;
	private static final boolean IS_INVERTED = true;
	private static final boolean SET_BRAKE = false;
	private static final int CURRENT_LIMIT = 20;

	// Gear ratio in SparkMAX is output / input (as opposed to input / output in CTRE)
	private static final double GEAR_RATIO = 1;
	private static final double MOMENT_OF_INERTIA = 0.001;

	private static void configMotor(SparkMaxMotor sparkMaxMotor) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(IS_INVERTED);
		config.smartCurrentLimit(CURRENT_LIMIT);

		config.encoder.positionConversionFactor(GEAR_RATIO);
		config.encoder.velocityConversionFactor(GEAR_RATIO);

		sparkMaxMotor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
		sparkMaxMotor.setBrake(SET_BRAKE);
	}

	private static BrushedSparkMAXMotor generateMotor(String logPath, SparkMaxWrapper sparkMaxWrapper) {
		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getNEO(NUMBER_OF_MOTORS), MOMENT_OF_INERTIA, 1 / GEAR_RATIO),
				DCMotor.getNEO(NUMBER_OF_MOTORS)
			)
		);

		BrushedSparkMAXMotor motor = new BrushedSparkMAXMotor(logPath, sparkMaxWrapper, simulation);
		configMotor(motor);
		return motor;
	}

	private static IDigitalInput generateLimitSwitch(BrushedSparkMAXMotor motor, String name) {
		if (RobotType.determineRobotType().isReal()) {
			return new SuppliedDigitalInput(
				motor.getSparkMaxWrapper().getForwardLimitSwitch()::isPressed,
				new Debouncer(0.02),
				ClimbConstants.IS_LIMIT_SWITCH_INVERTED
			);
		} else {
			return new ChooserDigitalInput(name);
		}
	}

	public static Solenoid createSolenoid(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXIDs.SOLENOID);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedDoubleSignal powerSignal = new SuppliedDoubleSignal("power", sparkMaxWrapper::get);

		BrushedSparkMAXMotor motor = generateMotor(logPath, sparkMaxWrapper);

		IDigitalInput limitSwitch = generateLimitSwitch(motor, "Climb limit switch");

		return new Solenoid(logPath, motor, voltageSignal, powerSignal, limitSwitch);
	}

}
