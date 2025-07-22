package frc.robot.vision.objectdetection;

public enum T2dEntryIndexes {

	DETECTED_HORIZONTAL_PIXELS(14),
	DETECTED_VERTICAL_PIXELS(15);

	private final int index;

	T2dEntryIndexes(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

}
