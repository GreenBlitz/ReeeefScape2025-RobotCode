package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.VelocityRequest;

public record DriveRequests(VelocityRequest velocity, IRequest<Double> voltage) {}
