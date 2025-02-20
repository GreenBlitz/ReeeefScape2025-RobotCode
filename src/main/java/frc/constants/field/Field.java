package frc.constants.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.Cage;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.utils.DriverStationUtil;
import frc.constants.field.enums.CoralStationSlot;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 17.548225;
	public static final double WIDTH_METERS = 8.0518;

	private static final Translation2d REEF_MIDDLE = new Translation2d(4.48934, 4.03225);

	public static final double REEF_SIDE_LENGTH_METERS = 0.96;

	private static final Pose2d[] REEF_SIDE_MIDDLES = new Pose2d[] {
		new Pose2d(3.65760, 4.03220, Rotation2d.fromDegrees(0)),
		new Pose2d(4.07349, 3.31191, Rotation2d.fromDegrees(60)),
		new Pose2d(4.90523, 3.31193, Rotation2d.fromDegrees(120)),
		new Pose2d(5.32107, 4.03225, Rotation2d.fromDegrees(180)),
		new Pose2d(4.90519, 4.75254, Rotation2d.fromDegrees(-120)),
		new Pose2d(4.07345, 4.75252, Rotation2d.fromDegrees(-60))};

	private static final Translation2d[] CORAL_BRANCHES = new Translation2d[] {
		new Translation2d(3.71123, 4.19654),
		new Translation2d(3.71008, 3.86792),
		new Translation2d(3.95799, 3.44052),
		new Translation2d(4.24201, 3.27522),
		new Translation2d(4.73610, 3.27621),
		new Translation2d(5.02126, 3.43953),
		new Translation2d(5.26745, 3.86792),
		new Translation2d(5.26859, 4.19654),
		new Translation2d(5.02069, 4.62394),
		new Translation2d(4.73667, 4.78924),
		new Translation2d(4.24258, 4.78825),
		new Translation2d(3.93194, 4.62493)};

	private static final Translation2d[] CAGES = new Translation2d[] {
		new Translation2d(8.77412, 7.26599),
		new Translation2d(8.77412, 6.17538),
		new Translation2d(8.77412, 5.08476)};

	public static final Translation2d BARGE_CENTER = new Translation2d(LENGTH_METERS / 2, WIDTH_METERS / 2);

	private static final Pose2d PROCESSOR = new Pose2d(5.98744, 0.00749, Rotation2d.fromDegrees(270));

	private static final int RIGHT_CORAL_STATION_ANGLE = 54;
	private static final int LEFT_CORAL_STATION_ANGLE = -54;

	private static final Pose2d[] CORAL_STATION_MIDDLES = new Pose2d[] {
		new Pose2d(0.84319, 0.65078, Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)),
		new Pose2d(0.84319, 7.41395, Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE))};

	private static final double Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES = 0.11941;
	public static final double FEEDER_WIDTH_METERS = 1.9304;

	public static final Pose2d[] CORAL_STATION_SLOTS_MIDDLES = new Pose2d[] {
		new Pose2d(
			0.18502,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R1.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.34944,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R2.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.51386,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R3.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.67828,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R4.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.8427,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R5.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.00712,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R6.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.7154,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R7.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.33596,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R8.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.50038,
			1.1278 - Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.R9.getIndex(),
			Rotation2d.fromDegrees(RIGHT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.18502,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L1.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.34944,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L2.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.51386,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L3.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.67828,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L4.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			0.8427,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L5.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.00712,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L6.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.7154,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L7.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.33596,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L8.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		),
		new Pose2d(
			1.50038,
			WIDTH_METERS - 1.1278 + Y_DIFFERENCE_BETWEEN_STATION_SLOTS_MIDDLES * CoralStationSlot.L9.getIndex(),
			Rotation2d.fromDegrees(LEFT_CORAL_STATION_ANGLE)
		)};

	public static Translation2d getReefMiddle() {
		return getAllianceRelative(REEF_MIDDLE, true, false);
	}

	public static Pose2d getReefSideMiddle(ReefSide side) {
		return getAllianceRelative(REEF_SIDE_MIDDLES[side.getIndex()], true, true, AngleTransform.INVERT);
	}

	public static Translation2d getCoralPlacement(Branch branch) {
		return getAllianceRelative(CORAL_BRANCHES[branch.getIndex()], true, true);
	}

	public static Translation2d getCage(Cage cage) {
		return getAllianceRelative(CAGES[cage.getIndex()], false, true);
	}

	public static Pose2d getProcessor() {
		return getAllianceRelative(PROCESSOR, true, true, AngleTransform.INVERT);
	}

	public static Pose2d getCoralStationMiddle(CoralStation coralStation) {
		return getAllianceRelative(CORAL_STATION_MIDDLES[coralStation.getIndex()], true, true, AngleTransform.INVERT);
	}

	public static Pose2d getCoralStationSlots(CoralStationSlot coralStationSlot) {
		return getAllianceRelative(
			CORAL_STATION_SLOTS_MIDDLES[coralStationSlot.isLeft() ? coralStationSlot.getIndex() + 9 : coralStationSlot.getIndex()],
			true,
			true,
			AngleTransform.INVERT
		);
	}

	public static Pose2d getAllianceRelative(Pose2d pose, boolean mirrorX, boolean mirrorY, AngleTransform angleTransform) {
		return isFieldConventionAlliance() ? pose : FieldMath.mirror(pose, mirrorX, mirrorY, angleTransform);
	}

	public static Translation2d getAllianceRelative(Translation2d translation, boolean mirrorX, boolean mirrorY) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, mirrorX, mirrorY);
	}

	public static Translation3d getAllianceRelative(Translation3d translation, boolean mirrorX, boolean mirrorY) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, mirrorX, mirrorY);
	}

	public static Rotation3d getAllianceRelative(Rotation3d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.mirrorAngle(rotation);
	}

	public static Rotation2d getAllianceRelative(Rotation2d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.mirrorAngle(rotation, AngleTransform.INVERT);
	}

	public static Pose3d getAllianceRelative(Pose3d pose, boolean mirrorX, boolean mirrorY, boolean mirrorAngle) {
		Translation3d translation3d = getAllianceRelative(pose.getTranslation(), mirrorX, mirrorY);
		return mirrorAngle ? new Pose3d(translation3d, getAllianceRelative(pose.getRotation())) : new Pose3d(translation3d, pose.getRotation());
	}

	public static Pose2d getPointFromCertainDistance(Pose2d point, double distantInMeters) {
		return new Pose2d(
			point.getX() - point.getRotation().getCos() * distantInMeters,
			point.getY() - point.getRotation().getSin() * distantInMeters,
			point.getRotation()
		);
	}


}
