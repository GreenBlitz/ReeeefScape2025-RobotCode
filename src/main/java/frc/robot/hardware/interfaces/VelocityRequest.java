package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface VelocityRequest extends IRequest<Rotation2d> {

	VelocityRequest withAcceleration(Rotation2d acceleration);

	Rotation2d getAcceleration();

	@Override
	VelocityRequest withSetPoint(Rotation2d setPoint);

}
