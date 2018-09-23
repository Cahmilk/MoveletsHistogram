package br.ufsc.trajectoryclassification.model.vo.features;

import java.util.HashMap;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class Nominal implements IFeature<Nominal> {
	
	private String value;
	
	private static Map<String, ProvidedDistances> providedDistances = new HashMap<>();
	
	public Nominal(String value) {
		this.value = value;		
	}
	
	public String getValue() {
		return value;
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return value;
	}

	@Override
	public double getDistanceTo(Nominal other, FeatureComparisonDesc featureComparisonDesc) {
		
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		case "equals": return this.value.equals(other.getValue()) ? 0 : 1 ;
		
		case "provided": return getDistanceFromProvided (
									this .getValue(), 
									other.getValue(), 
									featureComparisonDesc.getText()) ;
		
		case "weekdaydist": return weekdayDistance(this.value, other.getValue());		
		
		// equalsIgnoreCase
		case "equalsignoreccase": return this.value.equalsIgnoreCase(other.getValue()) ? 0 : 1 ;			
		
		default:
			break;
		}
		
		// TODO Auto-generated method stub
		return -1;
	}
	
	public boolean isWeekendDay(String value1){
		
		return value1.equalsIgnoreCase("Sunday") || value1.equalsIgnoreCase("Saturday");
		
	}
	
	public double weekdayDistance(String value1, String value2){
				
		if ( isWeekendDay(value1) && isWeekendDay(value2))
			return 0;
		
		else if ( !isWeekendDay(value1) && !isWeekendDay(value2))
			return 0;
		
		else 
			return 1;
		
	}
	
	public double getDistanceFromProvided(String value1, String value2, String providedName){
		
		/* Here we are seeking the provided name into the providadesDistances list and
		 * getting the distance value. 
		 * */
		return providedDistances.get(providedName).getDistance(value1, value2);		
		
	}
	
	public static Map<String, ProvidedDistances> getProvidedDistances() {
		return providedDistances;
	}

}
