package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class SPEED implements IPointFeature {	
	
	public SPEED() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculateSpeed(IPoint p1, IPoint p2){
		
		double distanceSpace = p1.getFeature("space").getDistanceTo(p2.getFeature("space"), null);
				
		double distanceTime = p1.getFeature("time").getDistanceTo(p2.getFeature("time"), null );
				
		return (distanceTime == 0) ? 0 : distanceSpace / distanceTime;		
		
	}
	

	@Override
	public void fillPoints(ITrajectory trajectory) {
		
		List<IPoint> points = trajectory.getData();
		
		/* The first element was not filled because it has not previous
		 * values to calculate speed, so its value is zero */
		points.get(0).getFeatures().put("speed", new Numeric(0));		
				
		for (int i = 1; i < points.size(); i++) {
			double speed = calculateSpeed(points.get(i-1), points.get(i));			
			points.get(i).getFeatures().put("speed", new Numeric( speed ));
			
			//System.out.println("Speed:" + speed);			
		}
		
	}

}
