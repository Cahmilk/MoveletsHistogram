package br.ufsc.trajectoryclassification.model.bo.dmbp;

import br.ufsc.trajectoryclassification.model.vo.IPoint;

public interface IDistanceMeasureBetweenPoints {

	public double[] getDistance(IPoint p1, IPoint p2);
	
	public double getDistance(IPoint p1, IPoint p2, int featureIndex);
	
	public int getNumberOfFeatures();
	
	public boolean haveMatch(IPoint p1, IPoint p2);

	public boolean haveMatch(IPoint p1, IPoint p2, double[] thresholds);

}
