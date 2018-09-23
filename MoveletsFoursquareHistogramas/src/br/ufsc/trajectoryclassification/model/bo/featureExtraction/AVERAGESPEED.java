package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.ArrayList;
import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class AVERAGESPEED implements ITrajectoryFeature, ISubtrajectoryFeature{
	
	public AVERAGESPEED() {
		// TODO Auto-generated constructor stub
	}

	public double getFeatureValue(ITrajectory trajectory) {

		double traveleddistance = 0;
		double timeduration = 0;

		/*
		 * Verify if traveled distance has been calculated
		 */
		if (trajectory.getFeatures().containsKey("traveleddistance"))
			traveleddistance = ((Numeric) trajectory.getFeature("traveleddistance")).getValue();
		else
			traveleddistance = new TRAVELEDDISTANCE().getFeatureValue(trajectory);

		/*
		 * Verifty if duration time has been calculated
		 */
		if (trajectory.getFeatures().containsKey("timeduration"))
			timeduration = ((Numeric) trajectory.getFeature("timeduration")).getValue();
		else
			timeduration = new TIMEDURATION().getFeatureValue(trajectory);

		return (timeduration == 0) ? 0 : traveleddistance / timeduration;
	}

	@Override
	public void fillTrajectory(ITrajectory trajectory) {
		
		trajectory.getFeatures().put("averagespeed", new Numeric( getFeatureValue(trajectory) ));
		
	}
	
		
	public double getFeatureValue(ISubtrajectory subtrajectory) {
		
		if (subtrajectory.getSize() == 1) return 0;
		
		double traveleddistance = 0;
		double timeduration = 0;

		/*
		 * Verify if traveled distance has been calculated
		 */
		if (subtrajectory.getFeatures().containsKey("traveleddistance"))
			traveleddistance = ((Numeric) subtrajectory.getFeature("traveleddistance")).getValue();
		else
			traveleddistance = new TRAVELEDDISTANCE().getFeatureValue(subtrajectory);

		/*
		 * Verifty if duration time has been calculated
		 */
		if (subtrajectory.getFeatures().containsKey("timeduration"))
			timeduration = ((Numeric) subtrajectory.getFeature("timeduration")).getValue();
		else
			timeduration = new TIMEDURATION().getFeatureValue(subtrajectory);

		return (timeduration == 0) ? 0 : traveleddistance / timeduration;
	}
	
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory){
		
		return new Numeric(getFeatureValue(subtrajectory));
		
	}

	@Override
	public void fillSubtrajectory(ISubtrajectory subtrajectory) {
		// TODO Auto-generated method stub
		subtrajectory.getFeatures().put("averagespeed", new Numeric( getFeatureValue(subtrajectory) ));
	}


}
