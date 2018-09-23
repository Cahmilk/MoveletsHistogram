package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class TRAVELEDDISTANCE implements ITrajectoryFeature, ISubtrajectoryFeature, IPointFeature {
	
	
	public TRAVELEDDISTANCE() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculate(IPoint p1, IPoint p2){
		
		return p1.getFeature("space").getDistanceTo(p2.getFeature("space"), null);		
		
	}
	
	public double getFeatureValue(ITrajectory trajectory){
		
		List<IPoint> points = trajectory.getData();
		
		double currentTraveledDistance = 0;
				
		for (int i = 1; i < points.size(); i++) {
			currentTraveledDistance += calculate(points.get(i-1), points.get(i));						
		}
		
		return currentTraveledDistance;
	}
	
	public double getFeatureValue(ISubtrajectory subtrajectory){
			
		List<IPoint> points = subtrajectory.getData();		
		
		if (points.get(0).getFeatures().containsKey("traveleddistance")){
			double startTraveledDistance = ((Numeric) points.get(0).getFeature("traveleddistance")).getValue();
			double endTraveledDistance = ((Numeric) points.get(points.size()-1).getFeature("traveleddistance")).getValue();
		
			return endTraveledDistance - startTraveledDistance;
		}			
					
		double currentTraveledDistance = 0;
				
		for (int i = 1; i < points.size(); i++) {
			currentTraveledDistance += calculate(points.get(i-1), points.get(i));						
		}
		
		return currentTraveledDistance;
	}
	

	@Override
	public void fillTrajectory(ITrajectory trajectory) {
	
		trajectory.getFeatures().put("traveleddistance", new Numeric( getFeatureValue(trajectory) ));
		
		//System.out.println("TraveledDistance:" + trajectory.getFeature("traveleddistance"));
	}

	@Override
	public void fillSubtrajectory(ISubtrajectory subtrajectory) {
		
		subtrajectory.getFeatures().put("traveleddistance", new Numeric( getFeatureValue(subtrajectory) ));
		
	}

	@Override
	public void fillPoints(ITrajectory trajectory) {

		List<IPoint> points = trajectory.getData();
		
		/* The first element was not filled because it has not previous
		 * values to calculate traveleddistance, so its value is zero */
		double traveledDistance = 0;
		points.get(0).getFeatures().put("traveleddistance", new Numeric(traveledDistance));		
		
		for (int i = 1; i < points.size(); i++) {
			traveledDistance += calculate(points.get(i-1), points.get(i));			
			points.get(i).getFeatures().put("traveleddistance", new Numeric( traveledDistance ));
		}
		
		
	}
	
	@Override
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory) {
		
		return new Numeric(getFeatureValue(subtrajectory));

	}

}
