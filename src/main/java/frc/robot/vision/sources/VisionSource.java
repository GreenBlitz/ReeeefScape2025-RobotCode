package frc.robot.vision.sources;

import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

import java.util.Optional;
import java.util.function.BiFunction;

public interface VisionSource<ReturnType extends VisionData> {

	void update();

	Optional<ReturnType> getVisionData();

	Optional<ReturnType> getFilteredVisionData();

	void setFilter(Filter<ReturnType> newFilter);

	Filter<ReturnType> getFilter();

	default void clearFilter() {
		setFilter(VisionFilters.getNonFilteringFilter());
	}

	default void applyOnExistingFilterWithNewFilter(
		BiFunction<Filter<ReturnType>, Filter<ReturnType>, Filter<ReturnType>> applicationFunction,
		Filter<ReturnType> filterToApplyWith
	) {
		setFilter(applicationFunction.apply(getFilter(), filterToApplyWith));
	}

}
