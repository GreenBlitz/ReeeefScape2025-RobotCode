package frc.robot.subsystems.flywheel;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Flywheel extends GBSubsystem {

    private final ControllableMotor rightMotor;
    private final ControllableMotor leftMotor;
    private final InputSignal[] rightSignals;
    private final InputSignal[] leftSignals;

    private final InputSignal rightVelocitySignal;
    private final InputSignal leftVelocitySignal;
    private final IRequest<Rotation2d> rightFlywheelVelocityRequest;
    private final IRequest<Rotation2d> leftFlywheelVelocityRequest;

    public Flywheel(
            String logPath,
            ControllableMotor rightFlywheel,
            ControllableMotor leftFlywheel,
            InputSignal rightVelocitySignal,
            InputSignal leftVelocitySignal,
            IRequest<Rotation2d> rightFlywheelRequest,
            IRequest<Rotation2d> leftFlywheelRequest,
            InputSignal[] rightSignals,
            InputSignal[] leftSignals
    ) {
        super(logPath);

        this.rightMotor = rightFlywheel;
        this.leftMotor = leftFlywheel;

        this.rightSignals = rightSignals;
        this.leftSignals = leftSignals;

        this.rightVelocitySignal = rightVelocitySignal;
        this.leftVelocitySignal = leftVelocitySignal;

        this.rightFlywheelVelocityRequest = rightFlywheelRequest;
        this.leftFlywheelVelocityRequest = leftFlywheelRequest;
    }


    @Override
    protected void subsystemPeriodic() {
        rightMotor.updateSignals(rightSignals);
        leftMotor.updateSignals(leftSignals);
    }

    public void setVelocities(Rotation2d rightFlywheelVelocity, Rotation2d leftFlywheelVelocity) {
        rightMotor.applyAngleRequest(rightFlywheelVelocityRequest.withSetPoint(rightFlywheelVelocity));
        leftMotor.applyAngleRequest(leftFlywheelVelocityRequest.withSetPoint(leftFlywheelVelocity));
    }

    public boolean
    isAtVelocities(Rotation2d rightFlywheelExpectedVelocity, Rotation2d leftFlywheelExpectedVelocity, Rotation2d velocityTolerance) {
        return isAtVelocities(rightFlywheelExpectedVelocity, leftFlywheelExpectedVelocity, velocityTolerance, velocityTolerance);
    }

    public void setPowers(double rightPower, double leftPower) {
        rightMotor.setPower(rightPower);
        leftMotor.setPower(leftPower);
    }

    public void stop() {
        setPowers(0, 0);
    }

    public boolean isAtVelocities(
            Rotation2d rightFlywheelExpectedVelocity,
            Rotation2d leftFlywheelExpectedVelocity,
            Rotation2d rightVelocityTolerance,
            Rotation2d leftVelocityTolerance
    ) {
        return MathUtil.isNear(
                rightFlywheelExpectedVelocity.getRotations(),
                rightVelocityTolerance.getRotations(),
                rightVelocityTolerance.getRotations()
        ) && MathUtil.isNear(
                leftFlywheelExpectedVelocity.getRotations(),
                leftVelocityTolerance.getRotations(),
                leftVelocityTolerance.getRotations()
        );
    }

}
