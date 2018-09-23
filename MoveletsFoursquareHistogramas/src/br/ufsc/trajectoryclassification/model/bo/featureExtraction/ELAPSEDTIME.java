package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class ELAPSEDTIME implements ITrajectoryFeature, ISubtrajectoryFeature, IPointFeature {
	
	
	public ELAPSEDTIME() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculate(IPoint p1, IPoint p2){
		
		return p1.getFeature("time").getDistanceTo(p2.getFeature("time"), null);		
		
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
		
		double currentTraveledDistance = 0;
				
		for (int i = 1; i < points.size(); i++) {
			currentTraveledDistance += calculate(points.get(i-1), points.get(i));						
		}
		
		return currentTraveledDistance;
	}
	

	@Override
	public void fillTrajectory(ITrajectory trajectory) {
	
		trajectory.getFeatures().put("Elapsedtime", new Numeric( getFeatureValue(trajectory) ));
		
		System.out.println("Elapsedtime:" + trajectory.getFeature("elapsedtime"));
	}

	@Override
	public void fillSubtrajectory(ISubtrajectory subtrajectory) {
		
		subtrajectory.getFeatures().put("elapsedtime", new Numeric( getFeatureValue(subtrajectory) ));
		
	}

	@Override
	public void fillPoints(ITrajectory trajectory) {

		List<IPoint> points = trajectory.getData();
		
		/* The first element was not filled because it has not previous
		 * values to calculate speed, so its value is zero */
		double elapsedtime = 0;
		points.get(0).getFeatures().put("elapsedtime", new Numeric(elapsedtime));		
		
		for (int i = 1; i < points.size(); i++) {
			elapsedtime += calculate(points.get(i-1), points.get(i));			
			points.get(i).getFeatures().put("elapsedtime", new Numeric( elapsedtime ));
		}
		
		
	}

	@Override
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory) {
		
		return new Numeric(getFeatureValue(subtrajectory));

	}

}
