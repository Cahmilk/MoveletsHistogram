package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.model.vo.features.Space2D;

public class TURNANGLE implements IPointFeature {	
	
	public TURNANGLE() {
		// TODO Auto-generated constructor stub
	}

	public double calculateAngle(IPoint p1, IPoint p2){
		
		Space2D p1Space = ((Space2D) p1.getFeature("space"));
		Space2D p2Space = ((Space2D) p2.getFeature("space"));
		
		double diffX = p2Space.getX() - p1Space.getX();  
		double diffY = p2Space.getY() - p1Space.getY();
		
		double angle = Math.toDegrees(Math.atan2(diffY,diffX));		
		
		return angle;		
		
	}

	public double calculateTurnangle(IPoint p1, IPoint p2, IPoint p3){
		
		double angle1 = calculateAngle(p1, p2);
		double angle2 = calculateAngle(p2, p3);
		double diff = angle2 - angle1;
		
		if (diff >  180) return diff - 360;
		if (diff < -180) return diff + 360;				
		return diff;		
		
	}
	

	@Override
	public void fillPoints(ITrajectory trajectory) {
		
		List<IPoint> points = trajectory.getData();
		
		/* The first element was not filled because it has not previous
		 * values to calculate turn angle, so its value is zero */
		points.get(0).getFeatures().put("turnangle", new Numeric(0));		
		points.get(points.size()-1).getFeatures().put("turnangle", new Numeric(0));		
				
		for (int i = 1; i < (points.size()-1); i++) {
			double turnangle = calculateTurnangle(points.get(i-1), points.get(i), points.get(i+1));			
			points.get(i).getFeatures().put("turnangle", new Numeric( turnangle ));	
		}
		
	}

}
