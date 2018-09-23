package br.ufsc.trajectoryclassification.model.bo.featureExtraction.xiao;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.apache.commons.math3.stat.descriptive.rank.Min;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.TRAVELEDDISTANCE;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.dodge.DodgeFeatures;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.zheng.ZhengFeatures;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.utils.ArrayIndexComparator;
import br.ufsc.trajectoryclassification.utils.Utils;

public class XiaoFeatures {

	/* Performs Point Feature Extraction */
	private List<String> strPointFeatures = 
			Arrays.asList(
					"traveleddistance",
					"speed",
					"acceleration",
					"turnangle",
					"sinuosity");
	
	
	private double thresholdHCR = 0;
	
	private double thresholdSR = 0;
	
	private double thresholdVCR = 0;
	
		
	public XiaoFeatures(double thresholdHCR, double thresholdSR, double thresholdVCR) {
		super();
		this.thresholdHCR = thresholdHCR;
		this.thresholdSR = thresholdSR;
		this.thresholdVCR = thresholdVCR;
	}

	public void fillTrajectory(ITrajectory trajectory, Description description){
		
		List<String> trajectoryFeaturesDesc = description.getTrajectoryFeaturesDesc();
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();

		features.putAll(getGlobalFeatures(trajectory));
		
		features.putAll(getLocalFeatures(trajectory));
		
		Utils.stopIfErrorValues(features);
				
		trajectory.getFeatures().putAll(features);
	}
	
	
	

	public  double getLength (ITrajectory t){

		if (!t.getFeatures().containsValue("traveleddistance"))
			new TRAVELEDDISTANCE().fillTrajectory(t);
		
		double distance = ((Numeric) t.getFeatures().get("traveleddistance")).getValue();
		
		return distance;
	}
	
	public static Map<String,IFeature> getGlobalFeatures(ITrajectory t, String strFeature){
		
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
		features.put("xiao_"+strFeature+"_mean", new Numeric(mean));
		features.put("xiao_"+strFeature+"_sd", new Numeric(sd));	
		double mode = getMode(valuesCopy, 100);
		features.put("xiao_"+strFeature+"_mode", new Numeric(mode));		
		features.put("xiao_"+strFeature+"_min1", new Numeric(valuesCopy[0]));
		features.put("xiao_"+strFeature+"_min2", new Numeric((n > 1) ? valuesCopy[1] : valuesCopy[0]));
		features.put("xiao_"+strFeature+"_min3", new Numeric((n > 2) ? valuesCopy[2] : valuesCopy[1]));
		features.put("xiao_"+strFeature+"_max1", new Numeric(valuesCopy[n-1]));
		features.put("xiao_"+strFeature+"_max2", new Numeric((n > 1) ? valuesCopy[n-2] : valuesCopy[n-1]));
		features.put("xiao_"+strFeature+"_max3", new Numeric((n > 2) ? valuesCopy[n-3] : valuesCopy[n-2]));		
		features.put("xiao_"+strFeature+"_range", new Numeric(valuesCopy[n-1]-values[0]));
		double percentile25 = ds.getPercentile(25);
		double percentile75 = ds.getPercentile(75);
		features.put("xiao_"+strFeature+"_percentile25", new Numeric(percentile25));
		features.put("xiao_"+strFeature+"_percentile75", new Numeric(percentile75));
		features.put("xiao_"+strFeature+"_interpercentil", new Numeric(percentile75-percentile25));
		double skewness = ds.getSkewness();
		double kurtosis = ds.getKurtosis();
		features.put("xiao_"+strFeature+"_skewness", new Numeric( (Double.isNaN(skewness)) ? 0 : skewness  ));
		features.put("xiao_"+strFeature+"_kurtosis", new Numeric( (Double.isNaN(kurtosis)) ? 0 : kurtosis ));
		features.put("xiao_"+strFeature+"_coefvariation", new Numeric( (mean != 0) ? sd / mean : 0 ));
		features.put("xiao_"+strFeature+"_autocorrleation", new Numeric( calculateAutocorrelation(valuesCopy, mean) ));
		
		for (String key : features.keySet()) {
			Double d = ((Numeric)features.get(key)).getValue();
			if (d.isInfinite() || d.isNaN() )
				System.out.println(t.getTid() + " " + key + " " + mean + " " + sd + " "+d);
		}
				
		return features;
	}
	
	public static double[] doubleToBins(double[] data, double min, double max, int numBins) {
		  final double[] result = new double[numBins];
		  final double binSize = (max - min)/numBins;

		  for (double d : data) {
			int bin = (int) Math.round(((d - min) / binSize));
		    if (bin < 0) { /* this data is smaller than min */ }
		    else if (bin >= numBins) { /* this data point is bigger than max */ }
		    else {
		      result[bin] += 1;
		    }
		  }
		  return result;
		}
	
	public static double getMode(double[] data, int numBins){
		double min = new Min().evaluate(data);
		double max = new Max().evaluate(data);
		double [] bins = doubleToBins(data,min,max,numBins);

		ArrayIndexComparator comparator = new ArrayIndexComparator(bins);
		Integer[] indexes = comparator.createIndexArray();
		Arrays.sort(indexes, comparator);
		
		double mode = min + bins[indexes[indexes.length-1]] * (max-min) / numBins;

		return mode;
	}
	
	public static double calculateAutocovariance(double values[], int k, double mean){
		
		double sum = 0;
		for (int i = 0; i < (values.length-k); i++) {
			sum += (values[i]-mean) * (values[i+k]-mean);
		}
		
		return (sum / values.length);		
	}
	
	public static double calculateAutocorrelation(double[] values, double mean){
		double c0 = calculateAutocovariance(values, 0, mean);
		double c1 = calculateAutocovariance(values, 1, mean);
		return (c0 != 0) ? (c1/c0) : 0;
	}
	
	
	
	public Map<String,IFeature> getGlobalFeatures(ITrajectory t){
			
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		/* Global features: statistical descriptors 17 X 4 = 68
		 * */
		features.putAll(getGlobalFeatures(t,"speed"));
		features.putAll(getGlobalFeatures(t,"acceleration"));
		features.putAll(getGlobalFeatures(t,"turnangle"));
		features.putAll(getGlobalFeatures(t,"sinuosity"));
		
		/* Global features: advanced features proposed by Zheng: 4
		 * */
		features.putAll(new ZhengFeatures(thresholdHCR, thresholdSR, thresholdVCR).getAdvancedFeatures(t));
				
		return features;
	}
	
	public Map<String,IFeature> getLocalFeatures(ITrajectory t){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		DodgeFeatures dodgeFeatures = new DodgeFeatures();
		
		features.putAll(dodgeFeatures.getProfileDecompositionFeatures(t, "speed"));
		features.putAll(dodgeFeatures.getProfileDecompositionFeatures(t, "acceleration"));
		features.putAll(dodgeFeatures.getProfileDecompositionFeatures(t, "turnangle"));
				
		return features;
	}
	
	
		
	
	public void fillAllTrajectories(List<ITrajectory> trajectories, Description description){

		new PointFeaturesExtraction().fillAllTrajectories(trajectories, strPointFeatures);		
	
		for (ITrajectory trajectory : trajectories) {
			fillTrajectory(trajectory, description);
		}		
		
	}

	

	
	
}
