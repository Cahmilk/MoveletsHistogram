package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class SINUOSITY implements IPointFeature {	
	
	public SINUOSITY() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculateSinuosity(IPoint p1, IPoint p2, IPoint p3){
		
		double distanceSpace1vs2 = p1.getFeature("space").getDistanceTo(p2.getFeature("space"), null);
		
		double distanceSpace2vs3 = p2.getFeature("space").getDistanceTo(p3.getFeature("space"), null);
		
		double distanceSpace1vs3 = p1.getFeature("space").getDistanceTo(p3.getFeature("space"), null);
		
		double sinuosity = (distanceSpace1vs3 != 0)	? (distanceSpace1vs2 + distanceSpace2vs3) / distanceSpace1vs3 : 0;	
				
		return sinuosity;		
		
	}
	

	@Override
	public void fillPoints(ITrajectory trajectory) {
		
		List<IPoint> points = trajectory.getData();
		
		/* The first element was not filled because it has not previous
		 * values to calculate speed, so its value is zero */
		points.get(0).getFeatures().put("sinuosity", new Numeric(0));
		points.get(points.size()-1).getFeatures().put("sinuosity", new Numeric(0));
				
		for (int i = 1; i < points.size()-1; i++) {
			double sinuosity = calculateSinuosity(points.get(i-1), points.get(i), points.get(i+1));			
			points.get(i).getFeatures().put("sinuosity", new Numeric( sinuosity ));
		}
		
	}

}
