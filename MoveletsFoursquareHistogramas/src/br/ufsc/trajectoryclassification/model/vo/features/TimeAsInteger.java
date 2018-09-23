package br.ufsc.trajectoryclassification.model.vo.features;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class TimeAsInteger implements IFeature<TimeAsInteger> {

	private int value;

	public TimeAsInteger(int value) {
		super();
		this.value = value;
	}
	
	public int getValue() {
		return value;
	}	
	
	@Override
	public double getDistanceTo(TimeAsInteger other, FeatureComparisonDesc featureComparisonDesc) {

		if (featureComparisonDesc == null)
			return Math.abs(this.value - other.getValue());		
		
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		case "difference": return Math.abs(this.value - other.getValue());			
		
		/* Other ways to compare 
		 * */		
		default: 
			break;
		}

		return -1;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj.getClass() != TimeAsInteger.class) 
			return false;
		
		return this.value == ((TimeAsInteger) obj).getValue();
	}
	
}
