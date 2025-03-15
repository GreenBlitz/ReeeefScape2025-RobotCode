package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import java.util.Comparator;
import java.util.Map;

public class InterpolationMap<T,K> extends InterpolatingTreeMap<T,K> {
	
	public InterpolationMap(InverseInterpolator<T> inverseInterpolator, Interpolator<K> interpolator, Map<T,K> dataPoints) {
		super(inverseInterpolator, interpolator);
		put(dataPoints);
	}
	
	public InterpolationMap(InverseInterpolator<T> inverseInterpolator, Interpolator<K> interpolator) {
		super(inverseInterpolator, interpolator);
	}
	
	public InterpolationMap(
			InverseInterpolator<T> inverseInterpolator,
			Interpolator<K> interpolator,
			Comparator<T> comparator) {
		super(inverseInterpolator,interpolator,comparator);
	}
	
	public void put(Map<T,K> dataPoints) {
		dataPoints.forEach(super::put);
	}
	
	public static Interpolator<Rotation2d> forRotation2d() {
		return (startValue, endValue, t) -> Rotation2d.fromDegrees(Interpolator.forDouble().interpolate(startValue.getDegrees(), endValue.getDegrees(), t));
	}
}
