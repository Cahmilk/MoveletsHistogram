package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.utils.Utils;
import weka.attributeSelection.AttributeEvaluator;
import weka.attributeSelection.InfoGainAttributeEval;
import weka.core.Attribute;
import weka.core.DenseInstance;
import weka.core.Instances;

public class LeftSidePureCV implements IQualityMeasure {

	private List<ITrajectory> trajectories;
	
	private List<String> labels;	
	
	private Map<String,Long> classes;
	
	private int samples = 1;
	
	private double sampleSize = 1;
	
	private String medium = "none";	
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
	
	public LeftSidePureCV(List<ITrajectory> trajectories, int samples, double sampleSize, String medium) {
		this.trajectories = trajectories;
		this.labels = new ArrayList<>();
		for (int j = 0; j < trajectories.size(); j++) {
			labels.add(trajectories.get(j).getLabel());
		}		
		this.classes = this.labels.stream().collect(Collectors.groupingBy(e -> e, Collectors.counting()));
		this.samples = samples;
		this.sampleSize = sampleSize;
		this.medium = medium;
	}
	
	
	private List<double[]> prunePoints(double[][] distances, String target, List<String> labels){
		
		List<Pair<MutableBoolean,double[]>> nonTargetDistances = new ArrayList<>();
		RealMatrix rm = new Array2DRowRealMatrix(distances);
		
		/* Select only the distance of the non-target label
		 * */
		for (int i = 0; i < labels.size(); i++) {
			if (!labels.get(i).equals(target))				
				nonTargetDistances.add( new Pair<>( new MutableBoolean(false), rm.getColumn(i)) );
		}				
						
		/* Iteratively remove all distances
		 * */
		Optional<Pair<MutableBoolean, double[]>> result;
		Pair<MutableBoolean, double[]> pair, pairToVerify;		
		boolean willRemove;
		
		result = nonTargetDistances.stream().findFirst();
				
		/* enquanto houver pares para explorar */
		while ( result.isPresent() ){
			
			pair = result.get();	
			pair.getFirst().setValue(true);
			
			for (int i = 0; i < nonTargetDistances.size(); i++) {
				
				pairToVerify = nonTargetDistances.get(i);
				
				willRemove = true;
				
				for (int j = 0; j < pair.getSecond().length; j++) {
					
					if (pairToVerify.getSecond()[j] <= pair.getSecond()[j]){
						willRemove = false;
						break; // for (int j = 0; j < distances.length; j++)
					}					
			
				
				} // for (int j = 0; j < distances.length; j++)
				
				
				if (willRemove) 
					nonTargetDistances.remove(i--);
				
			}
			
			result = nonTargetDistances.stream()
										.filter( (Pair<MutableBoolean,double[]> e) -> (e.getFirst().booleanValue() == false) )
										.findFirst();

		} // while ( result.isPresent() ){
		
		
		List<double[]> candidates = nonTargetDistances.stream().map(e -> e.getSecond()).collect(Collectors.toList());
				
		candidates.sort(new Comparator<double[]>() {
			@Override
			public int compare(double[] o1, double[] o2) {
				for (int i = 0; i < o1.length; i++) {
					if (o1[i] != o2[i])
						return (o1[i] > o2[i]) ? -1 : 1;
				}
				return 0;
			}
		});
		
		
		return candidates;
	}
	
	
	private List<double[]> createStairPoints(List<double[]> candidates){
		
		List<double[]> candidates2 = new ArrayList<>();
		int dimensions = candidates.get(0).length;
		for (int i = 0; i < (candidates.size()-dimensions+1); i++) {
			
			double[] maxFoundForDimensions = new double[dimensions]; 
			
			for (int j = 0; j < dimensions; j++) {
				
				maxFoundForDimensions[j] = candidates.get(i+0)[j];
				
				for (int k = 1; k < dimensions; k++) {
					if (candidates.get(i+k)[j] > maxFoundForDimensions[j])
						maxFoundForDimensions[j] = candidates.get(i+k)[j];
				}
				
			}
			
			candidates2.add(maxFoundForDimensions);			
		}
		
		return candidates2;
	}
	
	public double[][] clone2DArray(double [][] source){
		double[][] dest = new double[source.length][];
		for (int i = 0; i < dest.length; i++) {
			dest[i] = new double[source[i].length];
			for (int j = 0; j < dest[i].length; j++) {
				dest[i][j] = source[i][j];				
			}
		}
		return dest;		
	}
	
	private Pair<Integer,double[]> getBestSplitpoints(double[][] distances, String target, List<String> labels, List<double[]> candidates) {
		
		List<double[]> targetDistances = new ArrayList<>();
		RealMatrix rm = new Array2DRowRealMatrix(distances);
		
		/* Select only the distance of the non-target label
		 * */
		for (int i = 0; i < labels.size(); i++) {
			if (labels.get(i).equals(target))				
				targetDistances.add( rm.getColumn(i) );
		}
		
		double[] bestCandidate = new double[distances.length];
		int bestCount = 0;
		int currentCount;
		
		for (double[] currentCandidate : candidates) {
			currentCount = countCovered(targetDistances,currentCandidate);
			
			if (currentCount > bestCount){
				 bestCount = currentCount;
				 bestCandidate = currentCandidate;
			 }			
		}
						
		return new Pair<Integer,double[]>(bestCount,bestCandidate);
	}
	
	private Pair<double[][],List<String>> choosePoints(double[][] distances, List<String> labels, Random random){
			
		if (this.sampleSize == 1){
			return new Pair<>(distances,labels);
		}
		
		// Criar vetor de indices randomico
		int streamSize = (int) Math.round(labels.size() * this.sampleSize);		
		int[] intStream = random.ints(streamSize, 0, labels.size()).toArray();
		
		// Selecionar os dados
		double[][] newDistances = new double[distances.length][streamSize];
		List<String> newLabels = new ArrayList<>();
		
		for (int i = 0; i < intStream.length; i++) {
			for (int j = 0; j < newDistances.length; j++) {
				newDistances[j][i] = distances[j][intStream[i]];
			}
			newLabels.add(labels.get(intStream[i]));			
		}
		
		
		return new Pair<>(newDistances,newLabels);
	}
	
	private Map <String,double[]> getBestSplitpointsCV(double[][] distances, String target) {
		
		List<Pair<Integer,double[]>> results = new ArrayList<>();		
		Pair<double[][],List<String>> chosePoints = null;
					
		//Random random = new Random(1);
		Random random = new Random();
		
		for (int i = 0; i < this.samples; i++) {			
			
			chosePoints = choosePoints(distances, labels, random);			
			
			/* Step 1: Prune points of the opposite class
			 * */
			List<double[]> candidates = prunePoints(chosePoints.getFirst(), target, chosePoints.getSecond());
			
			/* Step 2: Create stair points, that are the real rectangles
			 * */
			List<double[]> stairCandidates = createStairPoints(candidates);
			
			/* Step 3: Choose the best rectangle
			 * */
			Pair<Integer,double[]> bestSplitpoints = getBestSplitpoints(chosePoints.getFirst(), target, chosePoints.getSecond(), stairCandidates);
			
			results.add(bestSplitpoints);
		}		
		
		// Agora resultado vai ser usado para acumular os resultados parciais
		int covered = 0;
		double[][] splitPoints = new double[distances.length][this.samples];
		
		Double minCovered = Double.MAX_VALUE;
		
		for (int i = 0; i < this.samples; i++) {
			
			for (int j = 0; j < results.get(i).getSecond().length; j++) {
				splitPoints[j][i] += results.get(i).getSecond()[j];
			}			
			if (results.get(i).getFirst() < minCovered)
				minCovered = results.get(i).getFirst() * 1.0d;
			covered += results.get(i).getFirst();
		}
		
		double[] splitPointsMean = new double[distances.length];
		double[] splitPointsSD = new double[distances.length];
		double[] splitPointsMin = new double[distances.length];
		double[] splitPointsMax = new double[distances.length];
		double[] splitPointsP25 = new double[distances.length];
		double[] splitPointsP75 = new double[distances.length];
		
		for (int i = 0; i < distances.length; i++) {
			DescriptiveStatistics ds = new DescriptiveStatistics(splitPoints[i]);
			splitPointsMean[i] = ds.getMean();
			splitPointsSD[i] = ds.getStandardDeviation();
			splitPointsMin[i] = ds.getMin();
			splitPointsMax[i] = ds.getMax();
			splitPointsP25[i] = ds.getPercentile(25);
			splitPointsP75[i] = ds.getPercentile(75);
		}
		
		Map <String,double[]> splitpointsData = new HashMap<>();
		splitpointsData.put("mean", splitPointsMean);
		splitpointsData.put("sd", splitPointsSD);
		splitpointsData.put("min", splitPointsMin);
		splitpointsData.put("max", splitPointsMax);
		splitpointsData.put("p25", splitPointsP25);
		splitpointsData.put("p75", splitPointsP75);
		
		
		return splitpointsData;
	}
	
	
	private boolean isCovered(double[] point, double[] limits){
		
		int dimensions = limits.length;
		
		for (int i = 0; i < dimensions; i++) {
			if (point[i] >= limits[i])
				return false;
		}
		
		return true;
	}
	
	private int countCovered(List<double[]> targetDistances, double[] candidate){
		
		int count = 0;
		
		for (int i = 0; i < targetDistances.size(); i++) {
			
			if (isCovered(targetDistances.get(i), candidate))
				count++;
			
		}
		
		return count;
	}
	
	public double getInformationGain(double[][] distances, List<String> labels, Map<String, double[]> splitpointsData){
		
		/* First we are going to discretize the distances into a 
		 * binary vector, where each value can assume two possible values
		 * 1. below the splitpoint
		 * 2. above or equal the splitpoint 
		 * */
		ArrayList<Attribute> atts = new ArrayList<Attribute>(2);
		ArrayList<String> strClasses = new ArrayList<String>(classes.keySet());
		ArrayList<String> strClassesNumber = new ArrayList<>();
		for (int i = 0; i < strClasses.size(); i++) {
			strClassesNumber.add(Integer.toString(i));
		}
		
        atts.add(new Attribute("distance", new ArrayList<String>(Arrays.asList("0","1")) ));
        atts.add(new Attribute("class", strClassesNumber));
        
		RealMatrix rm = new Array2DRowRealMatrix(distances);
						
		Instances dataRaw = new Instances("Dataset",atts,0);
        		
		for (int i = 0; i < distances[0].length; i++) {
			double[] instanceValue1 = new double[2];
            instanceValue1[0] = Utils.isCovered(rm.getColumn(i), splitpointsData.get("mean")) ? 0 : 1;
            instanceValue1[1] = strClasses.indexOf(labels.get(i));            
            dataRaw.add( new DenseInstance(1.0, instanceValue1) );
		}
		
		dataRaw.setClassIndex(1);
		
		/* Secondly we use the Information Gain algorithms of weka to get the information gain 
		 * */
		InfoGainAttributeEval evaluator = new InfoGainAttributeEval();
		double infogain = 0;
        try {			
        	evaluator.buildEvaluator(dataRaw);
			infogain = evaluator.evaluateAttribute(0);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		                
		return infogain;
	}
	
	
	public double getInformationGainMedium(double[][] distances, List<String> labels, Map<String, double[]> splitpointsData){
		
		/* First we are going to discretize the distances into a 
		 * binary vector, where each value can assume two possible values
		 * 1. below the splitpoint
		 * 2. above or equal the splitpoint 
		 * */
		ArrayList<Attribute> atts = new ArrayList<Attribute>(2);
		ArrayList<String> strClasses = new ArrayList<String>(classes.keySet());
		ArrayList<String> strClassesNumber = new ArrayList<>();
		for (int i = 0; i < strClasses.size(); i++) {
			strClassesNumber.add(Integer.toString(i));
		}
		
        atts.add(new Attribute("distance", new ArrayList<String>(Arrays.asList("0","1","2")) ));
        atts.add(new Attribute("class", strClassesNumber));
        
		RealMatrix rm = new Array2DRowRealMatrix(distances);
						
		Instances dataRaw = new Instances("Dataset",atts,0);
				
		Pair<double[], double[]> splitpointLimits = Utils.fillSplitPointsLimits(splitpointsData,medium);
		double[] splitpointsLI = splitpointLimits.getFirst(); // Limite inferior
		double[] splitpointsLS = splitpointLimits.getSecond(); // Limite superior

		for (int i = 0; i < distances[0].length; i++) {
			double[] instanceValue1 = new double[2];
			
			if (Utils.isCovered(rm.getColumn(i), splitpointsLI))
				instanceValue1[0] = 0;
			else if (Utils.isCovered(rm.getColumn(i), splitpointsLS))
				instanceValue1[0] = 1;
			else 
				instanceValue1[0] = 2;
			
            instanceValue1[1] = strClasses.indexOf(labels.get(i));            
            dataRaw.add( new DenseInstance(1.0, instanceValue1) );
		}
		
		dataRaw.setClassIndex(1);
		
		/* Secondly we use the Information Gain algorithms of weka to get the information gain 
		 * */
		InfoGainAttributeEval evaluator = new InfoGainAttributeEval();
		double infogain = 0;
        try {			
        	evaluator.buildEvaluator(dataRaw);
			infogain = evaluator.evaluateAttribute(0);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		                
		return infogain;
	}

	public void assesQuality(ISubtrajectory candidate) {
		// TODO Auto-generated method stub

		double[][] distances = candidate.getDistances();				
		String target = candidate.getTrajectory().getLabel();
		
		Map<String,double[]> splitpointsData = getBestSplitpointsCV(distances, target);
		
		/* Medir o ganho de informaçao dado esse split point
		 * 
		 * */
		double infogain = 0;
		
		switch (this.medium){
		
		case "none" : 
			infogain = getInformationGain(distances, this.labels, splitpointsData); 
			break;
		case "interquartil" : 
			infogain = getInformationGainMedium(distances, this.labels, splitpointsData); 
			break;
		case "sd" : 
			infogain = getInformationGainMedium(distances, this.labels, splitpointsData); 
			break;
		case "minmax" : 
			infogain = getInformationGainMedium(distances, this.labels, splitpointsData); 
			break;		
		case "default" : 
			infogain = getInformationGain(distances, this.labels, splitpointsData);
		}
		
		//infogain = (infogain < 0.05) ? 0 : infogain; 
				
		double[] maxDistances = getMaxDistances(distances);
				
		double dimensions = distances.length;
				
		/* It extracts several information about the quality
		 * */		
		Map<String, Double> data = new HashMap<>();
		
    	data.put("quality", infogain);
    	data.put("dimensions", 1.0 * dimensions );
    	data.put("size", 1.0 * candidate.getSize() );
    	data.put("start", 1.0 * candidate.getStart() );
    	data.put("tid", 1.0 * candidate.getTrajectory().getTid() );
    	
    	IQuality<LeftSidePureQuality> quality = new LeftSidePureQuality();
    	quality.setData(data);	    	
		candidate.setQuality(quality);
		candidate.setSplitpoints(splitpointsData.get("mean"));
		candidate.setSplitpointData(splitpointsData);
		candidate.setMaxDistances(maxDistances);
	}

	
	public void assesQuality3(ISubtrajectory candidate) {
		
		Map<String, Double> data = new HashMap<>();
    	data.put("quality", 0.0 );
    	data.put("count", 1.0);
    	data.put("splitpoint", 1.0);
    	data.put("size", 1.0 * candidate.getSize() );
    	data.put("start", 1.0 * candidate.getStart() );
    	data.put("tid", 1.0 * candidate.getTrajectory().getTid() );
    	    	
    	IQuality<LeftSidePureQuality> quality = new LeftSidePureQuality();
    	quality.setData(data);	    	
		candidate.setQuality(quality);
		
	}
	
	public void assesQuality2(ISubtrajectory candidate) {
		// TODO Auto-generated method stub
				
		double[][] distances = candidate.getDistances();
		
		String target = candidate.getTrajectory().getLabel();
		
		/* Step 1: Prune points of the opposite class
		 * */
		List<double[]> candidates = prunePoints(distances, target, this.labels);
		
		/* Step 2: Create stair points, that are the real rectangles
		 * */
		List<double[]> stairCandidates = createStairPoints(candidates);
		
		/* Step 3: Choose the best rectangel
		 * */
		Pair<Integer,double[]> bestSplitpoints = getBestSplitpoints(distances, target, this.labels, stairCandidates);
				
		double[] maxDistances = getMaxDistances(distances);
		
		int maxElementsForTheClass = classes.get(target).intValue();
		
		double maxDistance = 0;
		double splitdistance = 0;
		double splitpoint = 0;
				
		/* It considers the minimum number of trajectories covered by 
		 * a movelet as 3;
		 * */
		int count = 0;
		int dimensions = distances.length;
		int minCount = Math.round(maxElementsForTheClass * 0.1f);
		
		if (bestSplitpoints.getFirst() > minCount) {
			count = bestSplitpoints.getFirst()-1;
		}
	
		/* It extracts several information about the quality
		 * */		
		Map<String, Double> data = new HashMap<>();
    	data.put("quality", count / (double) (maxElementsForTheClass-1) );
    	data.put("count", 1.0 * count);
    	data.put("splitpoint", splitpoint);
    	data.put("splitpointIC", splitpoint);
    	data.put("splitdistance", splitdistance);
    	data.put("dimensions", 1.0 * dimensions );
    	data.put("size", 1.0 * candidate.getSize() );
    	data.put("start", 1.0 * candidate.getStart() );
    	data.put("tid", 1.0 * candidate.getTrajectory().getTid() );
    	data.put("maxDistance", maxDistance);
    	    	
    	IQuality<LeftSidePureQuality> quality = new LeftSidePureQuality();
    	quality.setData(data);	    	
		candidate.setQuality(quality);
		candidate.setSplitpoints(bestSplitpoints.getSecond());
		candidate.setMaxDistances(maxDistances);
	}


	private double[] getMaxDistances(double[][] distances) {
		
		double[] maxDistances = new double[distances.length];
		for (int i = 0; i < maxDistances.length; i++) {
			maxDistances[i] =
					Arrays.stream(distances[i]).filter(e -> e != Double.MAX_VALUE).max().getAsDouble();
		}
				
		return maxDistances;
	}



	
}
