package br.ufsc.trajectoryclassification.model.vo.features;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class Space2D implements IFeature<Space2D>{

	private double x;
	
	private double y;

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public Space2D(String str) {
		super();
		
		String[] row = str.split(" ");
		
		this.x = Double.parseDouble(row[0]);
		this.y = Double.parseDouble(row[1]);

	}
	
	public Space2D(double x, double y) {
		super();
		this.x = x;
		this.y = y;
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return "("+ x + " "+ y + ")";
	}

	
	public double euclideanDistance(Space2D one, Space2D other){
		
		double diffLat = Math.abs(one.getX() - other.getX());
		double diffLon = Math.abs(one.getY() - other.getY());
		
		return Math.sqrt( diffLat * diffLat + diffLon * diffLon );

	}
		
	public double manhattanDistance(Space2D one, Space2D other){
		
		double diffLat = Math.abs(one.getX() - other.getX());
		double diffLon = Math.abs(one.getY() - other.getY());
		
		return diffLat + diffLon;

	}
	
	public double normalizeDistance(double distance, double maxValue ){
		
		/* If maxValue was not defined
		 * */
		if (maxValue == -1)
			return distance;
				
		if (distance > maxValue)
			return Double.MAX_VALUE;
		
		return distance / maxValue;

	}
	
	@Override
	public double getDistanceTo(Space2D other, FeatureComparisonDesc featureComparisonDesc) {
		
		if (featureComparisonDesc == null){
			return euclideanDistance(this, other);		
		}

		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		case "euclidean": return normalizeDistance(euclideanDistance(this, other), featureComparisonDesc.getMaxValue());			
		
		case "manhattan": return normalizeDistance(manhattanDistance(this, other), featureComparisonDesc.getMaxValue());
		
		default:
			break;
		}
		
		// TODO Auto-generated method stub
		return -1;
	}


}
