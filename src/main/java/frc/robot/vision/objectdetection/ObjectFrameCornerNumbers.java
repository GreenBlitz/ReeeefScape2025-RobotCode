package frc.robot.vision.objectdetection;

public enum ObjectFrameCornerNumbers {

	TOP_LEFT(0),
	TOP_RIGHT(1),
	BOTTOM_RIGHT(2),
	BOTTOM_LEFT(3);

	private final int number;

	ObjectFrameCornerNumbers(int number) {
		this.number = number;
	}

	public int getNumber() {
		return number;
	}

}
