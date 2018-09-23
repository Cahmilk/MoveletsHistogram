package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.xiao.XiaoFeatures;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.utils.Utils;

public class FORTRANSPORTATIONMODE implements ITrajectoryFeature {

	/* Performs Point Feature Extraction */
	private List<String> strPointFeatures = 
			Arrays.asList(
					"traveleddistance",
					"speed",
					"acceleration");		
	
	@Override
	public void fillTrajectory(ITrajectory trajectory) {
		// TODO Auto-generated method stub
		
		new PointFeaturesExtraction().fillTrajectoryPoints(trajectory, strPointFeatures);
						
		Map<String,IFeature> features = new HashMap<String, IFeature>();

		features.putAll(getGlobalFeatures(trajectory));
		
		Utils.stopIfErrorValues(features);
				
		trajectory.getFeatures().putAll(features);
		
	}
	

	public Map<String,IFeature> getGlobalFeatures(ITrajectory t){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		/* Global features: statistical descriptors 17 X 4 = 68
		 * */
		features.putAll(getSimpleStatistics(t,"speed"));
		features.putAll(getSimpleStatistics(t,"acceleration"));
						
		return features;
	}

	
	public static Map<String,IFeature> getSimpleStatistics(ITrajectory t, String strFeature){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
				
		double[] values = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey(strFeature) ?  
						((Numeric) e.getFeatures().get(strFeature)).getValue() : Double.NaN).toArray();
		
		int n = values.length;
		
		double[] valuesCopy = Arrays.copyOf(values, n);
		Arrays.sort(valuesCopy);
		
		DescriptiveStatistics ds = new DescriptiveStatistics(valuesCopy);		
		
		double mean = ds.getMean();
		double sd = ds.getStandardDeviation();
		features.put("stats_"+strFeature+"_mean", new Numeric(mean));
		features.put("stats_"+strFeature+"_sd", new Numeric(sd));	

		for (String key : features.keySet()) {
			Double d = ((Numeric)features.get(key)).getValue();
			if (d.isInfinite() || d.isNaN() )
				System.out.println(t.getTid() + " " + key + " " + mean + " " + sd + " "+d);
		}
				
		return features;
	}
	
	public void fillAllTrajectories(List<ITrajectory> trajectories, Description description){

		new PointFeaturesExtraction().fillAllTrajectories(trajectories, strPointFeatures);		
		
		
		for (ITrajectory trajectory : trajectories) {
			fillTrajectory(trajectory);
		}		
		
	}
	
}
