package br.ufsc.trajectoryclassification.model.vo.features;

import java.time.DayOfWeek;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalField;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class TimeAsLocalDate implements IFeature<TimeAsLocalDate> {
	
	private LocalDate localDate;

	public LocalDate getLocalDate() {
		return localDate;
	}
	
	public void setLocalDate(LocalDate localDate) {
		this.localDate = localDate;
	}
	
	public boolean isWeekend(LocalDate localDate){
		// considera que os dias vao de 1 (Monday) ate 7 (Sunday)
		// Entao se o dia é maior ou igual a 6 (Satuday) 
		return ( localDate.getDayOfWeek().getValue() >= DayOfWeek.SATURDAY.getValue() );
	}
	
	public double areWeekendOrWeekday(LocalDate localDate1 , LocalDate localDate2){
		
		if ( isWeekend(localDate1) && isWeekend(localDate2)  )
			return 0;
		else if ( !isWeekend(localDate1) && !isWeekend(localDate2)  )
			return 0;
		else
			// Both days are not weekdays or weekend
			return 1;
	}
	
	@Override
	public double getDistanceTo(TimeAsLocalDate other, FeatureComparisonDesc featureComparisonDesc) {

		if (featureComparisonDesc == null)
			return ChronoUnit.MILLIS.between(this.getLocalDate(), other.getLocalDate()) / 1000;
			
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		case "difference": return ChronoUnit.MILLIS.between(this.localDate, other.getLocalDate()) / 1000;
		
		case "diffdayofweek": return ( this.localDate.getDayOfWeek() == other.getLocalDate().getDayOfWeek() ) ? 0 : 1; 
		
		case "equalsdayofweek": return ( this.localDate.getDayOfWeek() == other.getLocalDate().getDayOfWeek() ) ? 0 : 1;
		
		case "equalsweekdayorweekend": return areWeekendOrWeekday(localDate, other.localDate);
		
		
		
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
		return localDate.toString();
	}
}
