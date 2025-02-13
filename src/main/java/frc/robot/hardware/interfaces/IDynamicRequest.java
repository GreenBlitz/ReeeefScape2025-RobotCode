package frc.robot.hardware.interfaces;

public interface IDynamicRequest<T> extends IRequest<T>{
	IRequest<T> withVelocity(T setPoint);

	IRequest<T> withAcceleration(T setPoint);

}
