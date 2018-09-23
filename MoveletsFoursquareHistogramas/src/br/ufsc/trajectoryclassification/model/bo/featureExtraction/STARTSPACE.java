package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.model.vo.features.Space2D;

public class STARTSPACE  implements ITrajectoryFeature, ISubtrajectoryFeature{

	@Override
	public void fillSubtrajectory(ISubtrajectory subtrajectory) {
		// TODO Auto-generated method stub
		IFeature space2dStart = subtrajectory.getData().get(0).getFeature("space");
		subtrajectory.getFeatures().put("startspace", space2dStart );
	}

	@Override
	public void fillTrajectory(ITrajectory trajectory) {
		// TODO Auto-generated method stub
		IFeature space2dStart = trajectory.getData().get(0).getFeature("space");
		trajectory.getFeatures().put("startspace", space2dStart );
	}
	
	@Override
	public IFeature getIFeatureValue(ISubtrajectory subtrajectory) {
		
		return subtrajectory.getData().get(0).getFeature("space");

	}

}
