// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do not modify this file
 * except to change the parameter class to the startRobot call.
 */
public final class Main {

	/**
	 * Main initialization function. Do not perform any initialization here.
	 *
	 * <p>If you change your main robot class, change the parameter type.
	 */
	public static void main(String... args) {

		Rotation2d rot = Rotation2d.fromDegrees(-1);
		Rotation2d rot2 = Rotation2d.fromDegrees(-6);
		Rotation2d rot4 = Rotation2d.fromDegrees(145);

		double sumCos = rot.getCos() + rot2.getCos() + rot4.getCos();
		double sumSin = rot.getSin() + rot2.getSin() + rot4.getSin();

		double angleRad = Math.atan2(sumSin, sumCos);

		Rotation2d rotAvg = Rotation2d.fromRadians(angleRad);

		System.out.println(rotAvg);



//		RobotBase.startRobot(RobotManager::new);
	}

}
