package br.ufsc.trajectoryclassification.model.bo.featureExtraction.zheng;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.Variance;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.AVERAGESPEED;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.SPEED;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.TRAVELEDDISTANCE;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.utils.Utils;

public class ZhengFeatures {

	/* Performs Point Feature Extraction */
	private List<String> strPointFeatures = 
			Arrays.asList(
					"traveleddistance",
					"speed",
					"acceleration",
					"turnangle");
	
	
	private double thresholdHCR = 0;
	
	private double thresholdSR = 0;
	
	private double thresholdVCR = 0;
	
		
	public ZhengFeatures(double thresholdHCR, double thresholdSR, double thresholdVCR) {
		super();
		this.thresholdHCR = thresholdHCR;
		this.thresholdSR = thresholdSR;
		this.thresholdVCR = thresholdVCR;
	}

	public void fillTrajectory(ITrajectory trajectory, Description description){
		
		List<String> trajectoryFeaturesDesc = description.getTrajectoryFeaturesDesc();
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();

		features.putAll(getBasicFeatures(trajectory));
		
		features.putAll(getAdvancedFeatures(trajectory));
		
		for (String key : features.keySet()) {
			Double d = ((Numeric)features.get(key)).getValue();
			if (d.isInfinite() || d.isNaN() )
				System.out.println(trajectory.getTid() + " " + key + " ");
		}
		
		trajectory.getFeatures().putAll(features);
	}
	
	public static double getStopRate (ITrajectory t, double threshold){

		if (!t.getFeatures().containsValue("traveleddistance"))
			new TRAVELEDDISTANCE().fillTrajectory(t);

		double distance = ((Numeric) t.getFeatures().get("traveleddistance")).getValue();
		
		String strFeature = "speed";
		double[] values = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey(strFeature) ?  
						((Numeric) e.getFeatures().get(strFeature)).getValue() : Double.NaN).toArray();
		
		int n = values.length;
		
		double[] valuesCopy = Arrays.copyOf(values, n);
		Arrays.sort(valuesCopy);
		
		int i = 0;
		for (i = 0; i < valuesCopy.length; i++) {
			if (valuesCopy[i] > threshold)
				break;
		}
			
		double stopRate = (distance != 0) ? i / distance : 0; 
		
		return stopRate;
	}
	
	public  double getVelocityChangeRate (ITrajectory t, double threshold){

		if (!t.getFeatures().containsValue("traveleddistance"))
			new TRAVELEDDISTANCE().fillTrajectory(t);
		
		double distance = ((Numeric) t.getFeatures().get("traveleddistance")).getValue();
		
		String strFeature = "speed";
		double[] values = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey(strFeature) ?  
						((Numeric) e.getFeatures().get(strFeature)).getValue() : Double.NaN).toArray();
		
		int n = values.length;
		
		double[] valuesCopy = Arrays.copyOf(values, n);	
						
		int count = 0;
		for (int i = 0; i < (valuesCopy.length-1); i++) {
			double prop = Math.abs(valuesCopy[i+1]-valuesCopy[i]) / valuesCopy[i];
			if (prop > threshold) 
				count++;
		}
			
		double velocityChangeRate = (distance != 0) ? count / distance : 0; 
		
		return velocityChangeRate;
	}
	
	public  double getHeadingChangeRate (ITrajectory t, double threshold){

		if (!t.getFeatures().containsValue("traveleddistance"))
			new TRAVELEDDISTANCE().fillTrajectory(t);
		
		double distance = ((Numeric) t.getFeatures().get("traveleddistance")).getValue();
		
		String strFeature = "turnangle";
		double[] values = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey(strFeature) ?  
						((Numeric) e.getFeatures().get(strFeature)).getValue() : Double.NaN).toArray();
		
		int n = values.length;
		
		double[] valuesCopy = Arrays.copyOf(values, n);	
						
		int count = 0;
		for (int i = 0; i < valuesCopy.length; i++) {
			if (Math.abs(valuesCopy[i]) > threshold)				
				count++;
		}
			
		double headingChangeRate = (distance != 0) ? count / distance : 0; 
		
		return headingChangeRate;
	}
	
	
	public  double getLength (ITrajectory t){

		if (!t.getFeatures().containsValue("traveleddistance"))
			new TRAVELEDDISTANCE().fillTrajectory(t);
		
		double distance = ((Numeric) t.getFeatures().get("traveleddistance")).getValue();
		
		return distance;
	}
	
	public Map<String,IFeature> getAdvancedFeatures(ITrajectory t){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		/* Heading Change treshold: 20 degress
		 * */
		features.put("zheng_headingChangeRate", new Numeric(getHeadingChangeRate(t, thresholdHCR)));
		
		/* Velocity threshold: 10
		 * */
		features.put("zheng_stopRate", new Numeric(getStopRate(t, thresholdSR)));
		
		/* Velocity change threshold: 20%
		 * */				
		features.put("zheng_velocityChangeRate", new Numeric(getVelocityChangeRate(t, thresholdVCR)));
		
		features.put("zheng_length", new Numeric(getLength(t)));

		return features;
		
	}
	
	public static Map<String,IFeature> getBasicFeatures(ITrajectory t){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		/* The Basic Feature in Zheng 2008 are:
		 * ith Max values of speed: MaxV1, MaxV2, MaxV3, 
		 * ith Max values of acceleration: MaxA1, MaxA2, MaxA3, 
		 * Speed average: AV, 
		 * Expectation of points velocity: EV, (the mean of all points speeds)
		 * Variance of points velocity: DV
		 * Traveled Distance on the segment: Dist.
		 * */
		
		/* Preparing data
		 * */	
		double[] speedValues = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey("speed") ?  
						((Numeric) e.getFeatures().get("speed")).getValue() : Double.NaN).toArray();
		
		int ns = speedValues.length;
		
		double[] speedValuesCopy = Arrays.copyOf(speedValues, ns);
		Arrays.sort(speedValuesCopy);
		
		double[] accelerationValues = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey("acceleration") ?  
						((Numeric) e.getFeatures().get("acceleration")).getValue() : Double.NaN).toArray();
		
		int na = accelerationValues.length;
		
		double[] accelerationValuesCopy = Arrays.copyOf(accelerationValues, na);
		Arrays.sort(accelerationValuesCopy);
		
		/* Extracting feature values 
		 * */		
		double maxV1 = speedValuesCopy[ns-1];
		double maxV2 = (ns > 1) ? speedValuesCopy[ns-2] : maxV1;
		double maxV3 = (ns > 2) ? speedValuesCopy[ns-3] : maxV2;
		double maxA1 = accelerationValuesCopy[na-1];
		double maxA2 = (na > 1) ? accelerationValuesCopy[na-2] : maxA1;
		double maxA3 = (na > 2) ? accelerationValuesCopy[na-3] : maxA2;
		
		double AV = new AVERAGESPEED().getFeatureValue(t);
		double ES = new Mean().evaluate(speedValues);
		double DS = 0; new Variance().evaluate(speedValues);
		double dist = new TRAVELEDDISTANCE().getFeatureValue(t);
		
		features.put("zheng_"+"maxV1", new Numeric(maxV1));
		features.put("zheng_"+"maxV2", new Numeric(maxV2));
		features.put("zheng_"+"maxV3", new Numeric(maxV3));
		
		features.put("zheng_"+"maxA1", new Numeric(maxA1));
		features.put("zheng_"+"maxA2", new Numeric(maxA2));
		features.put("zheng_"+"maxA3", new Numeric(maxA3));
		
		features.put("zheng_"+"AV", new Numeric(AV));
		features.put("zheng_"+"ES", new Numeric(ES));
		features.put("zheng_"+"DS", new Numeric(DS));
		
		features.put("zheng_"+"dist", new Numeric(dist));
		
		Utils.stopIfErrorValues(features);
			
		return features;
	}
	
	
	

		
	
	public void fillAllTrajectories(List<ITrajectory> trajectories, Description description){

		new PointFeaturesExtraction().fillAllTrajectories(trajectories, strPointFeatures);		
	
		for (ITrajectory trajectory : trajectories) {
			fillTrajectory(trajectory, description);
		}		
		
	}

	

	
	
}
