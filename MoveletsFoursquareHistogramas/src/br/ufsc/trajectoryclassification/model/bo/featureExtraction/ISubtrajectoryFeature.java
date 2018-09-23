package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public interface ISubtrajectoryFeature {

	public void fillSubtrajectory(ISubtrajectory subtrajectory);
	
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory);
	
}

