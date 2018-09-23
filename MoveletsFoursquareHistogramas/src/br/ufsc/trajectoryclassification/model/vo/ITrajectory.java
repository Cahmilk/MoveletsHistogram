package br.ufsc.trajectoryclassification.model.vo;

import java.util.List;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.bo.dmbt.IDistanceMeasureBetweenTrajectories;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public interface ITrajectory{
	
	public double compareTo(ITrajectory otherPoint, IDistanceMeasureBetweenTrajectories dmbt);
	
	public Map<String, IFeature> getFeatures();
	
	public Map<String, Double> getAttributes();
	
	public List<IPoint> getData();

	public String getLabel();
	
	public int getTid();

	public IFeature getFeature(String featureName);


}
