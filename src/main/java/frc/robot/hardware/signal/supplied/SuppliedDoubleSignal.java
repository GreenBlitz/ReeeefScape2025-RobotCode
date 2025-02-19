package frc.robot.hardware.signal.supplied;

import frc.robot.hardware.signal.DoubleSignal;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.function.Supplier;

public class SuppliedDoubleSignal extends DoubleSignal {

	private final Supplier<Double> doubleSupplier;

	public SuppliedDoubleSignal(String name, Supplier<Double> doubleSupplier) {
		super(name);
		this.doubleSupplier = doubleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(doubleSupplier.get(), TimeUtil.getCurrentTimeSeconds());
	}

}
