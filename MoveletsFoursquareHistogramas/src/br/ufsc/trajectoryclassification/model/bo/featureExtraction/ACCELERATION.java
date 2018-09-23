package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class ACCELERATION implements IPointFeature {
	
	
	public ACCELERATION() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculateAcceleration(IPoint p1, IPoint p2){
		
		double differenceSpeed = 
				((Numeric) p2.getFeature("speed")).getValue() - ((Numeric) p1.getFeature("speed")).getValue();
				
		double differenceTime = p2.getFeature("time").getDistanceTo(p1.getFeature("time"), null );
				
		return (differenceTime != 0) ? differenceSpeed / differenceTime : 0;						
		
	}
	

	@Override
	public void fillPoints(ITrajectory trajectory) {
		
		List<IPoint> points = trajectory.getData();
		
		/* The first element was not filled because it has not previous
		 * values to calculate speed, so its value is zero */
		points.get(0).getFeatures().put("acceleration", new Numeric(0));		
				
		for (int i = 1; i < points.size(); i++) {
			
			double acceleration = calculateAcceleration(points.get(i-1), points.get(i));
			points.get(i).getFeatures().put("acceleration", new Numeric( acceleration ));
			
		}
		
	}

}
