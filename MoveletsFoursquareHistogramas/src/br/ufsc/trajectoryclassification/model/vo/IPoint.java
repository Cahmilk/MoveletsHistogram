package br.ufsc.trajectoryclassification.model.vo;

import java.util.HashMap;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public interface IPoint {
	
	public HashMap<String, IFeature> getFeatures();
	
	public IFeature getFeature(String featureName);
	
	public double[] compareTo(IPoint otherPoint, IDistanceMeasureBetweenPoints dmbp);
		
	public int getNumberOfValues();
	

}
