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
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.utils.ArrayIndexComparator;

public class LeftSidePureOld implements IQualityMeasure {

	private List<ITrajectory> trajectories;
	
	private List<String> labels;	
	
	private Map<String,Long> classes;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
	
	private Random random = new Random(1);
	
	
	public LeftSidePureOld(List<ITrajectory> trajectories) {
		this.trajectories = trajectories;
		this.labels = new ArrayList<>();
		for (int j = 0; j < trajectories.size(); j++) {
			labels.add(trajectories.get(j).getLabel());
		}
		this.classes = this.labels.stream().collect(Collectors.groupingBy(e -> e, Collectors.counting()));
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
		
		double[] bestCandidate = null;
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
	
	private Pair<double[][],List<String>> choosePoints(double[][] distances, List<String> labels){
			
		// Criar vetor de indices randomico
		int streamSize = Math.round(labels.size() * 0.6f);		
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
	
	private Pair<Integer,double[]> getBestSplitpointsCV(double[][] distances, String target) {
		
		List<Pair<Integer,double[]>> results = new ArrayList<>();		
		Pair<double[][],List<String>> chosePoints = null;
		int rep = 10;
						
		for (int i = 0; i < rep; i++) {			
			chosePoints = choosePoints(distances, labels);			
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
		double[] splitPoints = new double[distances.length];
		
		for (int i = 0; i < rep; i++) {
			
			if (results.get(i).getSecond() != null )
				
				for (int j = 0; j < results.get(i).getSecond().length; j++) {
					splitPoints[j] += results.get(i).getSecond()[j] * (1.0d/rep);
				}			
			
			covered += results.get(i).getFirst();
		}
		
		covered = covered / rep;
		
		return new Pair<>(covered,splitPoints);
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
	
	public void assesQuality1(ISubtrajectory candidate) {
		// TODO Auto-generated method stub

		double[][] distances = candidate.getDistances();				
		String target = candidate.getTrajectory().getLabel();
		
		Pair<Integer,double[]> bestSplitpoints = getBestSplitpointsCV(distances, target);
		
		/* Medir o ganho de informaçao dado esse split point
		 * 
		 * */
		
		
									
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
		int minCount = Math.round(maxElementsForTheClass * 0.2f);
		//int minCount = 3;
		
		if (bestSplitpoints.getFirst() > minCount) {
			count = bestSplitpoints.getFirst()-1;
		}
	
		count = bestSplitpoints.getFirst();
		/* It extracts several information about the quality
		 * */		
		Map<String, Double> data = new HashMap<>();
    	data.put("quality", count / (double) (maxElementsForTheClass-1) );
    	data.put("count", 1.0 * count);
    	data.put("splitpoint", splitpoint);
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

	
	
	
	public void assesQuality(ISubtrajectory candidate) {
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
	
		count = bestSplitpoints.getFirst();
		
		/* It extracts several information about the quality
		 * */		
		Map<String, Double> data = new HashMap<>();
    	data.put("quality", count / (double) (maxElementsForTheClass-1) );
    	data.put("count", 1.0 * count);
    	data.put("splitpoint", splitpoint);
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
