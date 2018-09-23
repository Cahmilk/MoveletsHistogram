package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;

import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Combinations;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class ParallelGetSubtrajectories implements Callable<List<ISubtrajectory>> {

	private ITrajectory trajectory;
	
	private List<ITrajectory> train;
	
	private int size;
	
	private double[][][][] mdist;
	
	private List<Integer> positions;
	
	private IDistanceMeasureForSubtrajectory dmbt;
		
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
	
	private boolean exploreDimensions;
	
	private IQualityMeasure qualityMeasure;

	private int maxNumberOfFeatures;
	
	public ParallelGetSubtrajectories(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist,
			List<Integer> positions, IDistanceMeasureForSubtrajectory dmbt, RankingAlgorithm rankingAlgorithm,
			boolean exploreDimensions, int maxNumberOfFeatures, IQualityMeasure qualityMeasure) {
		super();
		this.trajectory = trajectory;
		this.train = train;
		this.size = size;
		this.mdist = mdist;
		this.positions = positions;
		this.dmbt = dmbt;
		this.rankingAlgorithm = rankingAlgorithm;
		this.exploreDimensions = exploreDimensions;
		this.maxNumberOfFeatures = maxNumberOfFeatures;
		this.qualityMeasure = qualityMeasure;
	}


	@Override
	public List<ISubtrajectory> call() throws Exception {
		
		return run();		
		
	}
	
	
	private int getBestAlignmentByRanking(double[][] ranksForT, int[] comb) {
		
		double[] rankMerged = new double[ranksForT[0].length];
		for (int i = 0; i < comb.length; i++) {
			for (int j = 0; j < ranksForT[0].length; j++) {
				rankMerged[j] += ranksForT[comb[i]][j];
			}
		}
		/*			
		int minIdx = IntStream.range(0,rankMerged.length)
	            .reduce((i,j) -> rankMerged[i] > rankMerged[j] ? j : i)
	            .getAsInt(); 
	            */
		int minRankIndex = 0;
		for (int j = 1; j < rankMerged.length; j++) {
			if (rankMerged[j] < rankMerged[minRankIndex])
				minRankIndex = j;
		}
		/*
		System.out.println(Arrays.toString(rankMerged));
		System.out.println(minIdx);
		*/
		return minRankIndex;
	}


	private List<ISubtrajectory> getSubtrajectoriesForAPosition(int start){
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		int numberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();
		
		List<ISubtrajectory> list = SubtrajectoryBuilder.build(start, start + size - 1, trajectory, numberOfFeatures, train.size(), exploreDimensions, maxNumberOfFeatures);
		
		double[][][] distancesForAllT = mdist[start];
		
		double distance;
		
		for (int i = 0; i < train.size(); i++) {
			
			double[][] distancesForT = distancesForAllT[i];
			
			double[][] ranksForT = new double[distancesForT.length][];
			
			int limit = train.get(i).getData().size() - size + 1;				
			
			if (limit > 0)
				for (int k = 0; k < numberOfFeatures; k++) {				
					ranksForT[k] = rankingAlgorithm.rank(Arrays.stream(distancesForT[k],0,limit).toArray());
				} // for (int k = 0; k < numberOfFeatures; k++)
			
			int k2 = 0;
			
			int currentFeatures;
					
			
			if (exploreDimensions){
				currentFeatures = 1;
			} else {
				if (maxNumberOfFeatures > 1 && maxNumberOfFeatures < numberOfFeatures)
					currentFeatures = maxNumberOfFeatures;
				else
					currentFeatures = numberOfFeatures;
			}
			
			
			
			for (; currentFeatures <= numberOfFeatures; currentFeatures++) {
				
				for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) {
											
					int bestPosition = (limit > 0) ? getBestAlignmentByRanking(ranksForT,comb) : -1;
											
					for (int j = 0; j < comb.length; j++) {
						
						distance = (bestPosition >= 0) ? distancesForT[comb[j]][bestPosition] : Double.MAX_VALUE;
						
						list.get(k2).getDistances()[j][i] = 
								(distance != Double.MAX_VALUE) ? Math.sqrt(distance / size) 
															   : Double.MAX_VALUE;
						
					} // for (int j = 0; j < comb.length; j++)
											
					k2++;
					
				} // for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) 					
				
			} // for (int i = 0; i < train.size(); i++)
			
		} // for (int currentFeatures = 1; currentFeatures <= numberOfFeatures; currentFeatures++)
		
		candidates.addAll(list);
		
		return candidates;
	}

	private List<ISubtrajectory> run() {
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		// TODO Auto-generated method stub
		for (Integer position : positions) {
			candidates.addAll(getSubtrajectoriesForAPosition(position));
		}
		
		for (ISubtrajectory candidate : candidates) {
			qualityMeasure.assesQuality(candidate);
		}
		
		return candidates;
	}
	
	
	

}
