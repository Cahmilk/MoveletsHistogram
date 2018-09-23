package br.ufsc.trajectoryclassification.model.bo.featureExtraction.dodge;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;
import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.apache.commons.math3.stat.descriptive.summary.Sum;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;

public class DodgeFeatures {
	
	private double thresholdSinuosity = 0.5;
	
	/* Performs Point Feature Extraction */
	private List<String> strPointFeatures = 
			Arrays.asList(
					"traveleddistance",
					"speed",
					"acceleration",
					"turnangle",
					"sinuosity");
	
	
	public DodgeFeatures() {
		super();		
	}
	
	public DodgeFeatures(double thresholdSinuosity) {
		super();
		this.thresholdSinuosity = thresholdSinuosity;
	}

	public void fillTrajectory(ITrajectory trajectory, Description description){
		
		List<String> trajectoryFeaturesDesc = description.getTrajectoryFeaturesDesc();
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();

		/* In Dodge 2009, it is first extracted several descriptive statistics 
		 * (implemented in the function getGlobalFeatures) but after a feature
		 * selection process the author choose only mean and std (implemented
		 * in the function getSelectedGlobalFeatures). In additon, the author
		 * use initially 5 movement parameters, but after feature selection
		 * it uses only straightness index (sinuosity), speed, and acceleration.  
		 * */
		//features.putAll(getGlobalFeatures(trajectory));
		features.putAll(getSelectedGlobalFeatures(trajectory));
		
		features.putAll(getProfileDecompositionFeatures(trajectory));
				
		trajectory.getFeatures().putAll(features);
	}
	
	public Map<String,IFeature> getProfileDecompositionFeatures(ITrajectory trajectory){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		features.putAll(getProfileDecompositionFeatures(trajectory,"speed"));
		features.putAll(getProfileDecompositionFeatures(trajectory,"acceleration"));
		features.putAll(getProfileDecompositionFeatures(trajectory,"turnangle"));
		features.putAll(getProfileDecompositionFeatures(trajectory,"sinuosity"));
		
		return features;
	} 


	public Map<String,IFeature> getSelectedGlobalFeatures(ITrajectory trajectory){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		features.putAll(getSelectedGlobalFeatures(trajectory,"speed"));		
		features.putAll(getSelectedGlobalFeatures(trajectory,"acceleration"));
		features.putAll(getSelectedGlobalFeatures(trajectory,"sinuosity")); // or straightness		
		
		return features;
	} 

	
	public static Map<String,IFeature> getGlobalFeatures(ITrajectory trajectory){
		
		Map<String,IFeature> features = new HashMap<String, IFeature>();
		
		features.putAll(getGlobalFeatures(trajectory,"speed"));		
		features.putAll(getGlobalFeatures(trajectory,"acceleration"));
		features.putAll(getGlobalFeatures(trajectory,"turnangle"));
		features.putAll(getGlobalFeatures(trajectory,"traveleddistance"));
		features.putAll(getGlobalFeatures(trajectory,"sinuosity")); // or straightness		
		
		return features;
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
		double median = ds.getPercentile(50);
		double sd = ds.getStandardDeviation();
		double variance = ds.getVariance();
		double skewness = ds.getSkewness();
		features.put("dodge."+strFeature+".min", new Numeric(valuesCopy[0]));
		features.put("dodge."+strFeature+".max", new Numeric(valuesCopy[n-1]));
		features.put("dodge."+strFeature+".mean", new Numeric(mean));
		features.put("dodge."+strFeature+".median", new Numeric(median));	
		features.put("dodge."+strFeature+".sd", new Numeric(sd));	
		features.put("dodge."+strFeature+".variance", new Numeric(variance));				
		features.put("dodge."+strFeature+".skewness", new Numeric( (Double.isNaN(skewness)) ? 0 : skewness  ));
		
		for (String key : features.keySet()) {
			Double d = ((Numeric)features.get(key)).getValue();
			if (d.isInfinite() || d.isNaN() )
				System.out.println(t.getTid() + " " + key + " " + mean + " " + sd + " "+d);
		}
				
		return features;
	}

	public static Map<String,IFeature> getSelectedGlobalFeatures(ITrajectory t, String strFeature){
		
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
		features.put("dodge."+strFeature+".mean", new Numeric(mean));
		features.put("dodge."+strFeature+".sd", new Numeric(sd));	
		
		for (String key : features.keySet()) {
			Double d = ((Numeric)features.get(key)).getValue();
			if (d.isInfinite() || d.isNaN() )
				System.out.println(t.getTid() + " " + key + " " + mean + " " + sd + " "+d);
		}
				
		return features;
	}
	
	
	public Map<String,IFeature> getProfileDecompositionFeatures(ITrajectory t, String strFeature){

		Map<String,IFeature> features = new HashMap<String, IFeature>();
			
		int[] pc = getProfileDecomposition(t,strFeature);		
		
		for (int dc = 1; dc <= 4; dc++) {
			double[] pcFordc = getSegmentsForClass(pc, dc);
			double mean = (pcFordc.length > 0) ? new Mean().evaluate(pcFordc) : 0;			
			double sd =   (pcFordc.length > 1) ? new StandardDeviation().evaluate(pcFordc) : 0;			
			double prop = (pcFordc.length > 0) ? new Sum().evaluate(pcFordc) / pc.length : 0;			
			features.put("dodge_"+strFeature+"_meanlength_"+dc, new Numeric(mean) );
			features.put("dodge_"+strFeature+"_sdlength_"+dc, new Numeric(sd));			
			features.put("dodge._"+strFeature+"_prop_"+dc, new Numeric(prop));			
		}

		double changes = getNumberOfChanges(pc);
		features.put("dodge._"+strFeature+"_changes", new Numeric(changes));	

		return features;
	}
	
	public double[] getSegmentsForClass(int[] pc, int dc ){
		List<Double> segments = new ArrayList<>();
		
		for (int i = 0; i < pc.length; i++) {
			double length = 0;
			
			if (pc[i] == dc){
				length++;
				while (i < (pc.length-1) && pc[i] == pc[i+1]){
					i++;
					length++;
				}
			}
			
			if (length > 0) segments.add(length);
		}
		
		return segments.stream().mapToDouble(e->e).toArray();
	}
	
	public double getNumberOfChanges(int[] pc){
		
		double changes = 0;
		for (int i = 0; i < (pc.length-1); i++) {
			if (pc[i] != pc[i+1])
				changes++;
		}
		
		return changes;
	}
	
	
	public int[] getProfileDecomposition(ITrajectory t, String strFeature){		
		
		/* Get feature values
		 * */
		double[] values = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey(strFeature) ?  
						((Numeric) e.getFeatures().get(strFeature)).getValue() : Double.NaN).toArray();
		
		/* Get time values
		 * */
		double[] times = t.getData().stream().mapToDouble(
				e -> e.getFeatures().containsKey("time") ?  
						((Numeric) e.getFeatures().get("time")).getValue() : Double.NaN).toArray();
		
		/* Normalize time values by the sampling rate
		 * */
		double sum = 0;
		for (int i = 0; i < times.length-1; i++) {
			sum += times[i+1] - times[i];
		}
		final double samplingRate = sum / (times.length-1);		
		times = Arrays.stream(times).map(e -> e / samplingRate).toArray();
				
		/* Normalize feature values using min max
		 * */
		DescriptiveStatistics ds = new DescriptiveStatistics(values);		
		final double min = ds.getMin();
		final double max = ds.getMax();
		double[] normalizedValues = Arrays.stream(values).map(e -> (e - min) / (max-min)).toArray();
		
		/* Calculate the median of the values then to normalize 
		 * */
		int n = values.length;		
		double[] valuesCopy = Arrays.copyOf(normalizedValues, n);
		Arrays.sort(valuesCopy);
		double median = ((n % 2) == 1) ? valuesCopy[n/2-1] :  (valuesCopy[n/2-1] + valuesCopy[n/2]) / 2.0;
		
		/* Create deviation vector using normalized values and the median
		 * */
		double[] deviation = Arrays.stream(normalizedValues).map(e -> e - median).toArray();
		double dt = new StandardDeviation().evaluate(deviation);
		
		/* Create a sinuosity vector using normalized values
		 * */
		double[] sinuosity = new double[n];
		for (int i = 0; i < sinuosity.length; i++) {
			sinuosity[i] = calculateSinuosity(deviation,times,i);
			sinuosity[i] = 1 - Math.sqrt(1.0/(sinuosity[i] * sinuosity[i]));
		}
		/* Normalize sinuosity into the interval [0,1]
		 * */
		final double minSinuosity = new Min().evaluate(sinuosity);
		final double maxSinuosity = new Max().evaluate(sinuosity);
		sinuosity = Arrays.stream(sinuosity).map(e -> (e - minSinuosity) / (maxSinuosity-minSinuosity)).toArray();
		
		/* Calculate de profile decomposition (pc)
		 * */
		int[] pc = new int[n];
		for (int i = 0; i < pc.length; i++) {
			double s = sinuosity[i];
			double d = deviation[i];
			if (s < thresholdSinuosity && d < dt)
				pc[i] = 1; // low sinuosity and low deviation
			else if (s >= thresholdSinuosity && d < dt)
				pc[i] = 2; // high sinuosity and low deviation
			else if (s < thresholdSinuosity && d >= dt)
				pc[i] = 3; // low sinuosity and high deviation
			else 
				pc[i] = 4; // high siniosity and high deviation				
		}	
		
		return pc;
	}
		
	public double euclideanDistance( double x1, double y1, double x2, double y2){
		double dx = x2 - x1;
		double dy = y2 - y1;
		return Math.sqrt(dx * dx + dy * dy);
	}
		
	public double calculateSinuosityForALag(double[] values, double[] times, int p, int lag){
		
		int n = values.length;
		
		if ( (p-lag) < 0 || (p+lag) > (n-1)) return 1;
		
		double sum = 0;		
		for (int i = p-lag; i < (p+lag); i++) {
			sum += euclideanDistance(times[i], values[i], times[i+1], values[i+1]);
		}
						
		double beeline = euclideanDistance(times[p-lag], values[p-lag], times[p+lag], values[p+lag]);
		
		return sum / beeline;
	}
	
	public double calculateSinuosity(double[] values, double[] times, int p){
		double sinuosityLag1 = calculateSinuosityForALag(values, times, p, 1);		
		double sinuosityLag2 = calculateSinuosityForALag(values, times, p, 2);
		
		return (sinuosityLag1+sinuosityLag2) / 2.0;
	}
	
	public void fillAllTrajectories(List<ITrajectory> trajectories, Description description){
		
		new PointFeaturesExtraction().fillAllTrajectories(trajectories, strPointFeatures);		

		for (ITrajectory trajectory : trajectories) {
			fillTrajectory(trajectory, description);
		}		
		
	}

	
	
}
