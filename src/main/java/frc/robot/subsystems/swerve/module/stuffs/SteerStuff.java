package frc.robot.subsystems.swerve.module.stuffs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hard.interfaces.ControllableMotor;
import frc.robot.hard.interfaces.IRequest;
import frc.robot.hard.interfaces.InputSignal;

public record SteerStuff(
	ControllableMotor steer,
	IRequest<Rotation2d> positionRequest,
	IRequest<Double> voltageRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal<Rotation2d> velocitySignal,
	InputSignal<Double> currentSignal,
	InputSignal<Double> voltageSignal
) {}
