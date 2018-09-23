package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import org.apache.commons.collections4.ListUtils;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.model.vo.features.Space2D;

public class ENDSPACE implements ITrajectoryFeature, ISubtrajectoryFeature{

	@Override
	public void fillSubtrajectory(ISubtrajectory subtrajectory) {
		// TODO Auto-generated method stub
		IFeature space2dEnd = subtrajectory.getData().get(subtrajectory.getSize()-1).getFeature("space");
		subtrajectory.getFeatures().put("endspace", space2dEnd );
	}

	@Override
	public void fillTrajectory(ITrajectory trajectory) {
		// TODO Auto-generated method stub
		IFeature space2dEnd = trajectory.getData().get(trajectory.getData().size()-1).getFeature("space");
		trajectory.getFeatures().put("endspace", space2dEnd);
	}
	
	@Override
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory) {
		
		return subtrajectory.getData().get(subtrajectory.getSize()-1).getFeature("space");

	}
	
}
