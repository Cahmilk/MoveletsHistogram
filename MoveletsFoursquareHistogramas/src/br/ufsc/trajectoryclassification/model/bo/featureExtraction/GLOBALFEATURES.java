package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.xiao.XiaoFeatures;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.utils.Utils;

public class GLOBALFEATURES implements ITrajectoryFeature {

	/* Performs Point Feature Extraction */
	private List<String> strPointFeatures = 
			Arrays.asList(
					"traveleddistance",
					"speed",
					"acceleration",
					"turnangle",
					"sinuosity");
	
	
	
	@Override
	public void fillTrajectory(ITrajectory trajectory) {
		// TODO Auto-generated method stub
				
		Map<String,IFeature> features = new HashMap<String, IFeature>();

		features.putAll(getGlobalFeatures(trajectory));
		
		Utils.stopIfErrorValues(features);
				
		trajectory.getFeatures().putAll(features);
		
	}
	

	public Map<String,IFeature> getGlobalFeatures(ITrajectory t){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		/* Global features: statistical descriptors 17 X 4 = 68
		 * */
		features.putAll(XiaoFeatures.getGlobalFeatures(t,"speed"));
		features.putAll(XiaoFeatures.getGlobalFeatures(t,"acceleration"));
		features.putAll(XiaoFeatures.getGlobalFeatures(t,"turnangle"));
		features.putAll(XiaoFeatures.getGlobalFeatures(t,"sinuosity"));
						
		return features;
	}


	public void fillAllTrajectories(List<ITrajectory> trajectories, Description description){

		new PointFeaturesExtraction().fillAllTrajectories(trajectories, strPointFeatures);		
		
		for (ITrajectory trajectory : trajectories) {
			fillTrajectory(trajectory);
		}		
		
	}
	
}
