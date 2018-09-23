package br.ufsc.trajectoryclassification.model.vo.features;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public interface IFeature<T> {
	
	public double getDistanceTo(T other, FeatureComparisonDesc featureComparisonDesc);	

}
