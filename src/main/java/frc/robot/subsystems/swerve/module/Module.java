package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.swerve.module.extrainputs.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.module.extrainputs.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.module.stuffs.DriveStuff;
import frc.robot.subsystems.swerve.module.stuffs.EncoderStuff;
import frc.robot.subsystems.swerve.module.stuffs.SteerStuff;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Module {

	private final ModuleConstants constants;

	private final IAngleEncoder encoder;
	private final EncoderStuff encoderStuff;

	private final ControllableMotor steer;
	private final IRequest<Rotation2d> steerPositionRequest;
	private final IRequest<Double> steerVoltageRequest;
	private final SteerStuff steerStuff;

	private final ControllableMotor drive;
	private final IRequest<Rotation2d> driveVelocityRequest;
	private final IRequest<Double> driveVoltageRequest;
	private final DriveStuff driveStuff;

	private final ModuleInputsAutoLogged moduleInputs;
	private final DriveInputsAutoLogged driveInputs;

	private SwerveModuleState targetState;
	private Rotation2d startingSteerPosition;
	private boolean isClosedLoop;

	public Module(ModuleConstants constants, EncoderStuff encoderStuff, SteerStuff steerStuff, DriveStuff driveStuff) {
		this.constants = constants;

		this.encoder = encoderStuff.encoder();
		this.encoderStuff = encoderStuff;

		this.steer = steerStuff.steer();
		this.steerVoltageRequest = steerStuff.voltageRequest();
		this.steerPositionRequest = steerStuff.positionRequest();
		this.steerStuff = steerStuff;

		this.drive = driveStuff.drive();
		this.driveVoltageRequest = steerStuff.voltageRequest();
		this.driveVelocityRequest = driveStuff.velocityRequest();
		this.driveStuff = driveStuff;

		this.targetState = new SwerveModuleState();
		this.startingSteerPosition = new Rotation2d();
		this.isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
		this.moduleInputs = new ModuleInputsAutoLogged();
		this.driveInputs = new DriveInputsAutoLogged();

		updateInputs();
		resetByEncoder();
	}

	public SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo() {
		return steer.getSysidConfigInfo();
	}

	public SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo() {
		return drive.getSysidConfigInfo();
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}


	private void fixDriveInputsCoupling() {
		driveInputs.uncoupledVelocityPerSecond = ModuleUtils.uncoupleDriveAngle(
			driveStuff.velocitySignal().getLatestValue(),
			steerStuff.velocitySignal().getLatestValue(),
			constants.couplingRatio()
		);

		driveInputs.uncoupledPositions = new Rotation2d[driveStuff.positionSignal().asArray().length];
		for (int i = 0; i < driveInputs.uncoupledPositions.length; i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerStuff.positionSignal().asArray()[i].getRotations() - startingSteerPosition.getRotations());
			driveInputs.uncoupledPositions[i] = ModuleUtils
				.uncoupleDriveAngle(driveStuff.positionSignal().asArray()[i], steerDelta, constants.couplingRatio());
		}
	}

	public void updateInputs() {
		steer.updateSimulation();
		drive.updateSimulation();

		encoder.updateInputs(encoderStuff.positionSignal());
		steer.updateInputs(steerStuff.positionSignal(), steerStuff.velocitySignal(), steerStuff.currentSignal(), steerStuff.voltageSignal());
		drive.updateInputs(driveStuff.positionSignal(), driveStuff.velocitySignal(), driveStuff.currentSignal(), driveStuff.voltageSignal());

		fixDriveInputsCoupling();

		driveInputs.velocityMetersPerSecond = toDriveMeters(driveInputs.uncoupledVelocityPerSecond);
		driveInputs.positionsMeters = Arrays.stream(driveInputs.uncoupledPositions).mapToDouble(this::toDriveMeters).toArray();

		moduleInputs.isClosedLoop = isClosedLoop;
		moduleInputs.targetState = targetState;

		Logger.processInputs(constants.logPath(), moduleInputs);
		Logger.processInputs(driveStuff.logPath(), driveInputs);
	}


	public void setClosedLoop(boolean closedLoop) {
		isClosedLoop = closedLoop;
	}

	public void setBrake(boolean brake) {
		steer.setBrake(brake);
		drive.setBrake(brake);
	}

	public void resetByEncoder() {
		startingSteerPosition = encoderStuff.positionSignal().getLatestValue();
		steer.resetPosition(startingSteerPosition);
	}


	public void stop() {
		targetState = new SwerveModuleState(0, getSteerPosition());
		steer.stop();
		drive.stop();
	}


	public void setDriveVoltage(double voltage) {
		setClosedLoop(false);
		drive.applyRequest(driveVoltageRequest.withSetPoint(voltage));
	}

	public void setSteerVoltage(double voltage) {
		steer.applyRequest(steerVoltageRequest.withSetPoint(voltage));
	}


	public void pointSteer(Rotation2d steerTargetPosition, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, steerTargetPosition);
		targetState.angle = optimize ? targetState.optimize(moduleState, getSteerPosition()).angle : moduleState.angle;
		setTargetSteerPosition(targetState.angle);
	}


	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		this.targetState = SwerveModuleState.optimize(targetState, getSteerPosition());
		setTargetSteerPosition(this.targetState.angle);
		setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle, isClosedLoop);
	}

	private void setTargetSteerPosition(Rotation2d targetSteerPosition) {
		steer.applyRequest(steerPositionRequest.withSetPoint(targetSteerPosition));
	}

	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop) {
		targetVelocityMetersPerSecond = ModuleUtils.reduceSkew(targetVelocityMetersPerSecond, targetSteerPosition, getSteerPosition());

		if (isClosedLoop) {
			setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
		} else {
			setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
		}
	}

	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		setClosedLoop(true);
		Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(targetVelocityMetersPerSecond, constants.wheelDiameterMeters());
		Rotation2d coupledVelocityPerSecond = ModuleUtils
			.coupleDriveAngle(targetVelocityPerSecond, steerStuff.velocitySignal().getLatestValue(), constants.couplingRatio());
		drive.applyRequest(driveVelocityRequest.withSetPoint(coupledVelocityPerSecond));
	}

	public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
		double voltage = ModuleUtils.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			steerStuff.velocitySignal().getLatestValue(),
			constants.couplingRatio(),
			constants.velocityAt12VoltsPerSecond(),
			constants.wheelDiameterMeters(),
			ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
		);
		setDriveVoltage(voltage);
	}


	/**
	 * The odometry thread can update itself faster than the main code loop (which is 50 hertz). Instead of using the latest odometry update, the
	 * accumulated odometry positions since the last loop to get a more accurate position.
	 *
	 * @param odometryUpdateIndex the index of the odometry update
	 * @return the position of the module at the given odometry update index
	 */
	public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
		return new SwerveModulePosition(
			driveInputs.positionsMeters[odometryUpdateIndex],
			steerStuff.positionSignal().asArray()[odometryUpdateIndex]
		);
	}

	public int getNumberOfOdometrySamples() {
		return Math.min(driveInputs.positionsMeters.length, steerStuff.positionSignal().asArray().length);
	}

	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getSteerPosition());
	}

	public Rotation2d getDrivePosition() {
		return driveInputs.uncoupledPositions[driveInputs.uncoupledPositions.length - 1];
	}

	public double getDriveVelocityMetersPerSecond() {
		return driveInputs.velocityMetersPerSecond;
	}

	public Rotation2d getSteerPosition() {
		return steerStuff.positionSignal().getLatestValue();
	}


	public boolean isAtTargetVelocity(double speedToleranceMetersPerSecond) {
		return MathUtil.isNear(getTargetState().speedMetersPerSecond, getDriveVelocityMetersPerSecond(), speedToleranceMetersPerSecond);
	}

	public boolean isSteerAtTargetPosition(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		boolean isStopping = steerStuff.velocitySignal().getLatestValue().getRadians() <= steerVelocityPerSecondDeadband.getRadians();
		if (!isStopping) {
			return false;
		}
		boolean isAtSteerPosition = MathUtil.isNear(
			MathUtil.angleModulus(getTargetState().angle.getRadians()),
			MathUtil.angleModulus(getSteerPosition().getRadians()),
			steerTolerance.getRadians()
		);
		return isAtSteerPosition;
	}

	public boolean isAtTargetState(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband, double speedToleranceMetersPerSecond) {
		return isSteerAtTargetPosition(steerTolerance, steerVelocityPerSecondDeadband) && isAtTargetVelocity(speedToleranceMetersPerSecond);
	}

}
