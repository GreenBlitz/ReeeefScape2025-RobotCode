package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.AutonomousConstants;

import java.util.Optional;

public enum AutoPath {

	AL1_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE1, AutonomousConstants.LinkedWaypoints.I),
	AL1_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE1, AutonomousConstants.LinkedWaypoints.J),
	A_US(AutonomousConstants.LinkedWaypoints.A, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	B_LS(AutonomousConstants.LinkedWaypoints.B, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	C_LS(AutonomousConstants.LinkedWaypoints.C, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	D_LS(AutonomousConstants.LinkedWaypoints.D, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	E_LS(AutonomousConstants.LinkedWaypoints.E, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	F_LS(AutonomousConstants.LinkedWaypoints.F, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	G_LS(AutonomousConstants.LinkedWaypoints.G, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	H_US(AutonomousConstants.LinkedWaypoints.H, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	I_US(AutonomousConstants.LinkedWaypoints.I, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	K_US(AutonomousConstants.LinkedWaypoints.K, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	L_US(AutonomousConstants.LinkedWaypoints.L, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	LS_B(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.B),
	LS_C(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.C),
	LS_D(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.D),
	LS_E(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.E),
	LS_F(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.F),
	LS_G(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.G),
	US_A(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.A),
	US_H(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.H),
	US_I(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.I),
	US_J(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.J),
	US_K(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.K),
	US_L(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.L);

	private final Pair<String, Pose2d> startingPoint;
	private final Pair<String, Pose2d> endPoint;

	AutoPath(Pair<String, Pose2d> startingPoint, Pair<String, Pose2d> endPoint) {
		this.startingPoint = startingPoint;
		this.endPoint = endPoint;
	}

	public Pair<String, Pose2d> getStartingPoint() {
		return startingPoint;
	}

	public Pair<String, Pose2d> getEndPoint() {
		return endPoint;
	}

	public String getPathName() {
		return startingPoint.getFirst() + "-" + endPoint.getFirst();
	}

	public Optional<PathPlannerPath> getPath() {
		return PathPlannerUtil.getPathFromFile(getPathName());
	}

}
