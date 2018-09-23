package br.ufsc.trajectoryclassification.model.bo.dmbs;

import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;

public interface IDistanceMeasureForSubtrajectory {
		
	public Pair<ISubtrajectory,double[]> getBestAlignment(ISubtrajectory s, ITrajectory t);	
	
	//public double getDistance(ISubtrajectory s, ITrajectory t);	
	
	public double[] getDistance(ISubtrajectory s1, ISubtrajectory s2);	
	
	//public double getDistanceSubtrajectoryFeatures(ISubtrajectory s1, ISubtrajectory s2);
	
	public IDistanceMeasureBetweenPoints getDMBP();
	
	public Description getDescription();
	
}

