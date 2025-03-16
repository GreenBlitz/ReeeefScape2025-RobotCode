package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends GBSubsystem {

	private final ControllableMotor roller;
	private final InputSignal<Double> powerSignal;
	private final InputSignal<Double> currentSignal;
	private final IDigitalInput algaeLimitSwitch;
	private final DigitalInputInputsAutoLogged algaeLimitSwitchInputs;
	private final IDigitalInput coralBeamBreaker;
	private final DigitalInputInputsAutoLogged coralBeamBreakerInputs;
	private final EndEffectorCommandsBuilder commandsBuilder;
	private final RollerIOInputsAutoLogged inputs;
	private double calibrationPower = 0;

	public EndEffector(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> powerSignal,
		InputSignal<Double> currentSignal,
		IDigitalInput algaeLimitSwitch,
		IDigitalInput coralBeamBreaker
	) {
		super(logPath);
		this.roller = roller;
		this.powerSignal = powerSignal;
		this.currentSignal = currentSignal;

		this.algaeLimitSwitch = algaeLimitSwitch;
		this.algaeLimitSwitchInputs = new DigitalInputInputsAutoLogged();

		this.inputs = new RollerIOInputsAutoLogged();

		this.coralBeamBreaker = coralBeamBreaker;
		this.coralBeamBreakerInputs = new DigitalInputInputsAutoLogged();

		this.commandsBuilder = new EndEffectorCommandsBuilder(this);

		periodic();

		setDefaultCommand(commandsBuilder.setPower(0));
	}

	public EndEffectorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public boolean isAlgaeIn() {
		return algaeLimitSwitchInputs.debouncedValue;
	}

	public boolean isCoralIn() {
		return coralBeamBreakerInputs.debouncedValue;
	}

	public double getPower() {
		return powerSignal.getLatestValue();
	}

	public double getCurrent() {
		return currentSignal.getLatestValue();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		log();
	}

	@AutoLog
	public static class RollerIOInputs {

		public RollerIOData data = new RollerIOData(0, 0, false, false);

	}

	public record RollerIOData(double power, double current, boolean isAlgaeIn, boolean isCoralIn) {}

	private void updateInputs() {
		double time = TimeUtil.getCurrentTimeSeconds();
		roller.updateSimulation();
//		roller.updateInputs(powerSignal, currentSignal);
		inputs.data = new RollerIOData(powerSignal.getAndUpdateValue(), currentSignal.getAndUpdateValue(), isCoralIn(), isAlgaeIn());
		Logger.processInputs(getLogPath(), inputs);
		Logger.recordOutput("timetime/effecinput", TimeUtil.getCurrentTimeSeconds() - time);

//		algaeLimitSwitch.updateInputs(algaeLimitSwitchInputs);
//		Logger.processInputs(getLogPath() + "/AlgaeBeamBreaker", algaeLimitSwitchInputs);

		coralBeamBreaker.updateInputs(coralBeamBreakerInputs);
//		Logger.processInputs(getLogPath() + "/CoralBeamBreaker", coralBeamBreakerInputs);
	}

	private void log() {
//		Logger.recordOutput(getLogPath() + "/isAlgaeIn", isAlgaeIn());
//		Logger.recordOutput(getLogPath() + "/isCoralIn", isCoralIn());
	}

	public void setBrake(boolean brake) {
		roller.setBrake(brake);
	}

	protected void stop() {
		roller.stop();
	}

	protected void setPower(double power) {
		roller.setPower(power);
	}

	public void applyCalibrationsBindings(SmartJoystick joystick) {
		joystick.R1.whileTrue(commandsBuilder.setPower(() -> calibrationPower));

		joystick.B.onTrue(new InstantCommand(() -> calibrationPower = Math.max(calibrationPower - 0.01, -1)));
		joystick.X.onTrue(new InstantCommand(() -> calibrationPower = Math.min(calibrationPower + 0.01, 1)));
		joystick.A.onTrue(new InstantCommand(() -> calibrationPower = Math.max(calibrationPower - 0.1, -1)));
		joystick.Y.onTrue(new InstantCommand(() -> calibrationPower = Math.min(calibrationPower + 0.1, 1)));

		joystick.POV_LEFT.onTrue(commandsBuilder.setPower(EndEffectorState.DEFAULT.getPower()));
		joystick.POV_DOWN.onTrue(commandsBuilder.setPower(EndEffectorState.CORAL_INTAKE.getPower()).until(this::isCoralIn));
		joystick.POV_UP.onTrue(commandsBuilder.setPower(EndEffectorState.BRANCH_OUTTAKE.getPower()).until(() -> !isCoralIn()));
		joystick.POV_RIGHT.onTrue(commandsBuilder.stop());

		joystick.L1.onTrue(commandsBuilder.setPower(EndEffectorState.ALGAE_INTAKE.getPower()));
		joystick.R1.onTrue(commandsBuilder.setPower(EndEffectorState.ALGAE_OUTTAKE.getPower()));
	}

}
