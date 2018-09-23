package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class TIMEDURATION implements ITrajectoryFeature, ISubtrajectoryFeature {
	
	
	public TIMEDURATION() {
		// TODO Auto-generated constructor stub
	}
		
	public double calculate(IPoint p1, IPoint p2){
		
		return p1.getFeature("time").getDistanceTo(p2.getFeature("time"), null);
		
	}
	
	public double getFeatureValue(ITrajectory trajectory){
		
		List<IPoint> points = trajectory.getData();
		
		IPoint firstPoint = points.get(0);
		IPoint lastPoint = points.get(points.size()-1);
		
		return calculate(firstPoint,lastPoint);
	}
	
	public double getFeatureValue(ISubtrajectory subtrajectory){
		
		List<IPoint> points = subtrajectory.getData();
		
		IPoint firstPoint = points.get(0);
		IPoint lastPoint = points.get(points.size()-1);
		
		return calculate(firstPoint,lastPoint);
	}
	

	@Override
	public void fillTrajectory(ITrajectory trajectory) {

		trajectory.getFeatures().put("timeduration", new Numeric( getFeatureValue(trajectory) ));
		
		//System.out.println("TimeDuration:" + trajectory.getFeature("timeduration"));
	}

	@Override
	public void fillSubtrajectory(ISubtrajectory subtrajectory) {

		subtrajectory.getFeatures().put("timeduration", new Numeric( getFeatureValue(subtrajectory) ));
				
	}
	
	@Override
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory) {
		
		return new Numeric(getFeatureValue(subtrajectory));

	}

}
