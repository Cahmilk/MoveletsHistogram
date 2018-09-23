package br.ufsc.trajectoryclassification.model.bo.dmbt;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Trajectory;

public interface IDistanceMeasureBetweenTrajectories {

	public double getDistance(ITrajectory t1, ITrajectory t2);
	
	public IDistanceMeasureBetweenPoints getDistanceMeasureBetweenPoints();
	
	public void setDistanceMeasureBetweenPoints(IDistanceMeasureBetweenPoints dmbp);
	
	public void setThresholds(double [] values);
	
	public void setWeights(double [] values);
	
	public Object clone() throws CloneNotSupportedException;
	
}
