package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import org.apache.commons.lang3.StringUtils;
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

public class LeftSidePureCVLigth implements IQualityMeasure {

	private List<ITrajectory> trajectories;
	
	private List<String> labels;	
	
	private Map<String,Long> classes;
	
	private int samples = 1;
	
	private double sampleSize = 1;
	
	private String medium = "none";	
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
	
	public LeftSidePureCVLigth(List<ITrajectory> trajectories, int samples, double sampleSize, String medium) {
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
	
	
	private boolean firstVectorGreaterThanTheSecond(double [] first, double [] second){
		
		for (int i = 0; i < first.length; i++) {
			if (first[i] <= second[i])
				return false;
		}
		
		return true;
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
		
		nonTargetDistances.sort(new Comparator<Pair<MutableBoolean, double[]>>() {
			@Override
			public int compare(Pair<MutableBoolean, double[]> o1, Pair<MutableBoolean, double[]> o2) {
								
				for (int i = 0; i < o1.getSecond().length; i++) {
					if ( (i % 2) == 0){
						if (o1.getSecond()[i] < o2.getSecond()[i])
							return -1;
						else if (o2.getSecond()[i] < o1.getSecond()[i])
							return +1;
					} else {
						if (o1.getSecond()[i] > o2.getSecond()[i])
							return -1;
						else if (o2.getSecond()[i] > o1.getSecond()[i])
							return +1;						
					}
							
				}
					
				return 0;
				
			}
		});
		
		/* Iteratively remove all distances
		 * */
		Optional<Pair<MutableBoolean, double[]>> result;
		Pair<MutableBoolean, double[]> pair, pairToVerify;		
		boolean willRemove;
						
		// Remove candidates with DoubleMax values
		for (int i = 0; i < nonTargetDistances.size(); i++) {
			if ( new DescriptiveStatistics( nonTargetDistances.get(i).getSecond()).getMax() == Double.MAX_VALUE)				
				nonTargetDistances.remove(i--);
		}
		
		/* Remove repeated candidates
		 * */
		for (int i = 1; i < nonTargetDistances.size(); i++) {
			if (Arrays.equals(nonTargetDistances.get(i-1).getSecond(),
					nonTargetDistances.get(i).getSecond() ))
				
				nonTargetDistances.remove(i--);
		}
		
		result = nonTargetDistances.stream().findFirst();
		
		/* enquanto houver pares para explorar */
		while ( result.isPresent() ){
			
			pair = result.get();	
			pair.getFirst().setValue(true);
			
			for (int i = 0; i < nonTargetDistances.size(); i++) {
				
				pairToVerify = nonTargetDistances.get(i);
				
				willRemove = firstVectorGreaterThanTheSecond(pairToVerify.getSecond(), pair.getSecond());
								
				if (willRemove) 
					nonTargetDistances.remove(i--);
				
			}
			
			result = nonTargetDistances.stream()
										.filter( (Pair<MutableBoolean,double[]> e) -> (e.getFirst().booleanValue() == false) )
										.findFirst();

		} // while ( result.isPresent() ){
		
		
		List<double[]> candidates = nonTargetDistances.stream().map(e -> e.getSecond()).collect(Collectors.toList());
			/*
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
		*/
		/*
		Set<Integer> indexes = new HashSet<Integer>();		
		for (int i = 0; i < candidates.size(); i++) {
			for (int j = 0; j < rm.getRowDimension(); j++) {
				if (Arrays.equals(candidates.get(i), rm.getRow(j)));
					indexes.add(j);
			}			
		}	
		System.out.println(StringUtils.join(indexes,", "));
		System.out.println();
		*/
		
		return candidates;
	}
	
	
	private List<double[]> createStairPoints(List<double[]> candidates){
		
		if (candidates.isEmpty())
			return candidates;
		
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
	
	private Pair<double[][],List<String>> choosePointsStratified(double[][] distances, List<String> labels, String target, Random random){
		
		if (this.sampleSize == 1){
			return new Pair<>(distances,labels);
		}
		
		List<Integer> positive = new ArrayList<>();
		List<Integer> negative = new ArrayList<>();
		for (int i = 0; i < labels.size(); i++) {
			if (labels.get(i).equals(target))
				positive.add(i);
			else
				negative.add(i);
		}
		
		List<Integer> choosed = new ArrayList<>();
		Collections.shuffle(positive,random);
		choosed.addAll(positive.subList(0, (int) (positive.size() * this.sampleSize) ) );		
		Collections.shuffle(negative,random);
		choosed.addAll(negative.subList(0, (int) (negative.size() * this.sampleSize) ) );
		
		// Selecionar os dados
		double[][] newDistances = new double[distances.length][choosed.size()];
		List<String> newLabels = new ArrayList<>();
		
		for (int i = 0; i < choosed.size(); i++) {
			for (int j = 0; j < newDistances.length; j++) {
				newDistances[j][i] = distances[j][choosed.get(i)];
			}
			newLabels.add(labels.get(choosed.get(i)));			
		}
		
		
		return new Pair<>(newDistances,newLabels);
	}
	
	private Map<String,double[]> getBestSplitpointsCV(double[][] distances, String target) {
		
		List<Pair<Integer,double[]>> results = new ArrayList<>();		
		Pair<double[][],List<String>> chosePoints = null;
					
		//Random random = new Random(1);
		Random random = new Random();
		
		for (int i = 0; i < this.samples; i++) {			
			
			//chosePoints = choosePoints(distances, labels, random);
			chosePoints = choosePointsStratified(distances, labels, target, random);
			
			/* Step 1: Prune points of the opposite class
			 * */
			List<double[]> candidates = prunePoints(chosePoints.getFirst(), target, chosePoints.getSecond());
			
			/* Step 2: Create stair points, that are the real rectangles
			 * */
			//List<double[]> stairCandidates = createStairPoints(candidates);
			List<double[]> stairCandidates = candidates;
						
			/* Step 3: Choose the best rectangle
			 * */
			Pair<Integer,double[]> bestSplitpoints = getBestSplitpoints(chosePoints.getFirst(), target, chosePoints.getSecond(), stairCandidates);
			
			results.add(bestSplitpoints);
		}		
		
		// Agora resultado vai ser usado para acumular os resultados parciais
		double[][] splitPoints = new double[distances.length][this.samples];
		
		Double minCovered = Double.MAX_VALUE;
		
		for (int i = 0; i < this.samples; i++) {
			
			for (int j = 0; j < results.get(i).getSecond().length; j++) {
				splitPoints[j][i] += results.get(i).getSecond()[j];
			}			
			
			if (results.get(i).getFirst() < minCovered)
				minCovered = results.get(i).getFirst() * 1.0d;
			
		}
		

		double[] splitPointsMean = new double[distances.length];		
		
		for (int i = 0; i < distances.length; i++) {
			DescriptiveStatistics ds = new DescriptiveStatistics(splitPoints[i]);
			splitPointsMean[i] = ds.getMean();
		}
		
		Map <String,double[]> splitpointsData = new HashMap<>();
		splitpointsData.put("mean", splitPointsMean);		
		
		return splitpointsData;
	}
	
	/*	
	private boolean isCovered(double[] point, double[] limits){
		
		int dimensions = limits.length;
		
		for (int i = 0; i < dimensions; i++) {
			if (point[i] >= limits[i])
				return false;
		}
		
		return true;
	}*/
	
	
	/*
	 * Para considerar empate por conta de valores discretos
	 */
	private boolean isCovered(double[] point, double[] limits){
		
		int dimensions = limits.length;
		
		for (int i = 0; i < dimensions; i++) {
			if (limits[i] > 0){
				if (point[i] >= limits[i])
					return false;
			}
			else
				if (point[i] > limits[i])
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
	
	
	public double getFMeasure(double[][] distances, List<String> labels, Map<String, double[]> splitpointsData, String target){
		
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
		        
		int tp = 0, tn = 0, fp = 0, fn = 0;
		boolean covered;
		
		for (int i = 0; i < distances[0].length; i++) {
			//covered = Utils.isCovered(rm.getColumn(i), splitpointsData.get("mean"));
			covered = isCovered(rm.getColumn(i), splitpointsData.get("mean"));
			if (covered){				
				if ( target.equals( labels.get(i) ))
					tp++;
				else
					fp++;				
			} else {

				if ( target.equals( labels.get(i) ))
					fn++;
				else
					tn++;
			}
		}
		
		if (tp == 1) return 0;
		
		double precision = (tp + fp) > 0 ? tp / ((tp + fp) * 1.0d) : 0.0;
		double recall 	 = (tp + fn) > 0 ? tp / ((tp + fn) * 1.0d) : 0.0;
		double f1		 = (precision > 0 && recall > 0) ? 2.0 / (1/precision + 1/recall) : 0.0;
		
		return f1;		
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
		
		double f1 = 0;
				
		f1 = getFMeasure(distances,this.labels, splitpointsData,target);
						
		double[] maxDistances = getMaxDistances(distances);
				
		double dimensions = distances.length;
				
		/* It extracts several information about the quality
		 * */		
		Map<String, Double> data = new HashMap<>();
		
    	data.put("quality", f1);
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


	private double[] getMaxDistances(double[][] distances) {
		
		double[] maxDistances = new double[distances.length];
		for (int i = 0; i < maxDistances.length; i++) {
			maxDistances[i] =
					Arrays.stream(distances[i]).filter(e -> e != Double.MAX_VALUE).max().getAsDouble();
		}
				
		return maxDistances;
	}



	
}
