package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class STAYTIME implements IPointFeature {
	
	
	public STAYTIME() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculate(IPoint p1, IPoint p2){
		
		double diffTime = p1.getFeature("time").getDistanceTo(p2.getFeature("time"), null); 
		
		return diffTime / 2;		
		
	}
	
	public double getFeatureValue(ITrajectory trajectory){
		
		List<IPoint> points = trajectory.getData();
		
		double currentTraveledDistance = 0;
				
		for (int i = 1; i < points.size(); i++) {
			currentTraveledDistance += calculate(points.get(i-1), points.get(i));						
		}
		
		return currentTraveledDistance;
	}

	@Override
	public void fillPoints(ITrajectory trajectory) {

		List<IPoint> points = trajectory.getData();
		
		/* According to Patel we have 3 options to measure the weight wi for a point pi
		 * wi = (t2-t1), i = 1 (first)
		 * wi = (tn-tn-1), i = N (last)
		 * wi = (ti-ti+1), otherwise
		 * */
		int n = points.size();
		double staytimeFirst = calculate(points.get(0), points.get(1));
		double staytimeLast  = calculate(points.get(n-1), points.get(n-2));
		points.get(0).getFeatures().put("staytime", new Numeric(staytimeFirst));
		points.get(n-1).getFeatures().put("staytime", new Numeric(staytimeLast));
		
		for (int i = 1; i < (points.size()-1); i++) {
			double staytime = calculate(points.get(i-1), points.get(i+1));			
			points.get(i).getFeatures().put("staytime", new Numeric(staytime));
		}
					
	}
	

}
