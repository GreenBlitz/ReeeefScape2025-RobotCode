package frc.constants.field.enums;

import frc.utils.pose.Side;

public enum ReefSide {

	A(0),
	B(1),
	C(2),
	D(3),
	E(4),
	F(5);

	private final int index;

	ReefSide(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static ReefSide getReefSideBySideAndFar(Side side, boolean isFar) {
		return switch (side) {
			case LEFT -> isFar ? E : F;
			case MIDDLE -> isFar ? D : A;
			case RIGHT -> isFar ? C : B;
		};
	}

}
