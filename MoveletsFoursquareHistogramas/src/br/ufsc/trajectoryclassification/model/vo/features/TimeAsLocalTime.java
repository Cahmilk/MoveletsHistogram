package br.ufsc.trajectoryclassification.model.vo.features;

import java.time.DayOfWeek;
import java.time.LocalTime;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalField;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class TimeAsLocalTime implements IFeature<TimeAsLocalTime> {
	
	private LocalTime localTime;

	public TimeAsLocalTime(LocalTime localTime) {
		super();
		this.localTime = localTime;
	}
	
	public LocalTime getLocalTime() {
		return localTime;
	}

	@Override
	public double getDistanceTo(TimeAsLocalTime other, FeatureComparisonDesc featureComparisonDesc) {

		if (featureComparisonDesc == null)
			return ChronoUnit.MILLIS.between(this.localTime, other.getLocalTime()) / 1000;
			
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		case "difference": return ChronoUnit.MILLIS.between(this.localTime, other.getLocalTime()) / 1000;
		
		/* Other ways to compare 
		 * */		
		default: 
			break;
		}

		return -1;
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return localTime.toString();
	}
}
