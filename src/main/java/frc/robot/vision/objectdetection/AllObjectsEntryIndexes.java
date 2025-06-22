package frc.robot.vision.objectdetection;

public enum AllObjectsEntryIndexes {

	TARGET_ID(0),
	TX_NO_CROSS(1),
	TY_NO_CROSS(2),
	TARGET_AREA(3),
	TOP_LEFT_CORNER_X(4),
	TOP_LEFT_CORNER_Y(5),
	TOP_RIGHT_CORNER_X(6),
	TOP_RIGHT_CORNER_Y(7),
	BOTTOM_RIGHT_CORNER_X(8),
	BOTTOM_RIGHT_CORNER_Y(9),
	BOTTOM_LEFT_CORNER_X(10),
	BOTTOM_LEFT_CORNER_Y(11);

	private final int index;

	AllObjectsEntryIndexes(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
