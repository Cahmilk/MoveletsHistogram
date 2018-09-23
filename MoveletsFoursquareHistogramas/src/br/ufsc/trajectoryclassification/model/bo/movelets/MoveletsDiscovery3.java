package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.stream.IntStream;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Combinations;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;


public class MoveletsDiscovery3 implements Callable<Integer> {
	
	private List<ISubtrajectory> candidates;

	private ITrajectory trajectory;
	
	private List<ITrajectory> trajectories;

	private IDistanceMeasureForSubtrajectory dmbt;
	
	private IQualityMeasure qualityMeasure;
	
	private int minSize;
	
	private int maxSize;
	
	private boolean cache;
	
	private boolean exploreDimensions;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
			
	public MoveletsDiscovery3(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions) {
		super();
		this.candidates = candidates;
		this.trajectory = trajectory;
		this.trajectories = trajectories;
		this.dmbt = dmbt;
		this.qualityMeasure = qualityMeasure;
		this.minSize = minSize;
		this.maxSize = maxSize;
		this.cache = cache;
		this.exploreDimensions = exploreDimensions;
	}

	@Override
	public Integer call() throws Exception {		
	
		measureShapeletCollection();
	
		return 0;

	}

	public void measureShapeletCollection() {
		
		List<ISubtrajectory> candidates;
		
		if (this.cache)
			candidates = fastMoveletsDiscoveryUsingCache(trajectory, trajectories, minSize, maxSize);
		else 
			candidates = moveletsDiscoveryWithoutCache(trajectory, trajectories, minSize, maxSize);

		for (ISubtrajectory candidate : candidates) {
			/*
			 * STEP 1: COMPUTE DISTANCES, IF NOT COMPUTED YET
			 */
			if (candidate.getDistances() == null)		
				ComputeDistances(candidate);
			
			/*
			 * STEP 2: ASSES QUALITY, IF REQUIRED
			 */
			if (qualityMeasure != null & candidate.getQuality() != null)
				AssesQuality(candidate);
		}
		
		/* STEP 3: SELECTING BEST CANDIDATES
		 * */		
		this.candidates.addAll(MoveletsFilterAndRanker.getShapelets(candidates));
		//this.candidates.addAll(candidates);
	
	}

	
	private void AssesQuality(ISubtrajectory candidate) {
		qualityMeasure.assesQuality(candidate);
	}

	private void ComputeDistances(ISubtrajectory candidate) {
		
		/* This pairs will store the subtrajectory of the best alignment 
		 * of the candidate into each trajectory and the distance 
		 * */
		Pair<ISubtrajectory,double[]> distance;
		
		double[][] trajectoryDistancesToCandidate = new double[candidate.getSplitpoints().length]
															  [trajectories.size()];
		
		ISubtrajectory[] bestAlignments = new ISubtrajectory[trajectories.size()];
				
		/* It calculates the distance of trajectories to the candidate
		 */
		for (int i = 0; i < trajectories.size(); i++) {
			
			distance = dmbt.getBestAlignment(candidate, trajectories.get(i));
						
			bestAlignments[i] = distance.getFirst();
			trajectoryDistancesToCandidate[i] = distance.getSecond();			
		}
		
		candidate.setDistances(trajectoryDistancesToCandidate);
		candidate.setBestAlignments(bestAlignments);
	}

	private List<ISubtrajectory> getCandidatesUsingMDist(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist) {
		
		int n = trajectory.getData().size();
				
		List<ISubtrajectory> candidates = new ArrayList<>();

		for (int start = 0; start <= (n - size); start++) {
			
			int numberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();
			
			List<ISubtrajectory> list = SubtrajectoryBuilder.build(start, start + size - 1, trajectory, numberOfFeatures, train.size(), exploreDimensions, 3);
			
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

		} // for (int start = 0; start <= (n - size); start++)
		
		return candidates;
		
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

	public int getArrayIndex(double[] arr,double value) {
	    for(int i=0;i<arr.length;i++)
	        if(arr[i]==value) return i;
	    return -1;
	}
	
	private boolean[][][][] getBaseCaseUncomparedDim( double[][][][] baseCase){
		
		boolean[][][][] baseCasedUncomparedDim = new boolean[baseCase.length][][][]; 
				
		for (int i = 0; i < baseCase.length; i++) {
			baseCasedUncomparedDim[i] = new boolean[baseCase[i].length][][];
			for (int j = 0; j < baseCasedUncomparedDim[i].length; j++) {
				baseCasedUncomparedDim[i][j] = new boolean[baseCase[i][j].length][];
				for (int k = 0; k < baseCasedUncomparedDim[i][j].length; k++) {
					baseCasedUncomparedDim[i][j][k] = new boolean[baseCase[i][j][k].length];
					for (int k2 = 0; k2 < baseCasedUncomparedDim[i][j][k].length; k2++) {
						if (baseCase[i][j][k][k2] == Double.MAX_VALUE)
							baseCasedUncomparedDim[i][j][k][k2] = true;
						else 
							baseCasedUncomparedDim[i][j][k][k2] = false;
					}
				}
			}
		}
		
		return baseCasedUncomparedDim;
		
	}
	
	private double[][][][] getBaseCase(ITrajectory trajectory, List<ITrajectory> train){
		int n = trajectory.getData().size();
		int size = 1;
		
		double[][][][] base = new double[(n - size)+1][][][];		
		
		for (int start = 0; start <= (n - size); start++) {
			
			base[start] = new double[train.size()][][];				
			
			for (int i = 0; i < train.size(); i++) {
								
				base[start][i] = new double[dmbt.getDMBP().getNumberOfFeatures()][(train.get(i).getData().size()-size)+1];
						
				for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
					
					double[] distance = dmbt.getDMBP().getDistance(
							trajectory.getData().get(start),
							train.get(i).getData().get(j));
							
					for (int k = 0; k < distance.length; k++) {
					
						base[start][i][k][j] = (distance[k] != Double.MAX_VALUE) ? (distance[k] * distance[k]) : Double.MAX_VALUE;					
					
					} // for (int k = 0; k < distance.length; k++)
					
				} // for (int j = 0; j <= (train.size()-size); j++)
				
			} //for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)

		return base;
	}


	private double[][][][] getNewSize(ITrajectory trajectory, List<ITrajectory> train, double[][][][] base, double[][][][] lastSize, int size, boolean[][][][] baseUncomparedDim) {
		
		int n = trajectory.getData().size();	
		
		for (int start = 0; start <= (n - size); start++) {
						
			for (int i = 0; i < train.size(); i++) {
				
				if (train.get(i).getData().size() >= size) {						
							
					for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
												
						for (int k = 0; k < lastSize[start][i].length; k++) {
							
							if (lastSize[start][i][k][j] != Double.MAX_VALUE)
								
								if ( ! baseUncomparedDim[start+size-1][i][k][j+size-1])
									
									lastSize[start][i][k][j] += base[start+size-1][i][k][j+size-1];
						
						} // for (int k = 0; k < distance.length; k++) {
											
					} // for (int j = 0; j <= (train.size()-size); j++)
					
				} // if (train.get(i).getData().size() >= size) 
				
			} // for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)
		
		return lastSize;
	}


	private List<ISubtrajectory> moveletsDiscoveryWithoutCache(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize){
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		int n = trajectory.getData().size();
		maxSize = (maxSize == -1) ? n : maxSize;
				
		MyCounter.numberOfCandidates += (maxSize * (maxSize-1) / 2);
		
		int numberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();
				
		for (int size = minSize; size <= maxSize; size++) {				
			
			List<ISubtrajectory> candidatesOfSize = new ArrayList<>();
			
			for (int start = 0; start <= (n - size); start++) {				
												
				List<ISubtrajectory> list = SubtrajectoryBuilder.build(start, start + size - 1, trajectory, numberOfFeatures, train.size(), exploreDimensions, 3);
								
				MoveletsFinding m = new MoveletsFinding(list, train, dmbt);
				//ComputeDistances(candidate);
				
				for (ISubtrajectory candidate : list) {
					candidate.setDistances(null);
					m.ComputeDistances(candidate);
					//AssesQuality(candidate);
				}							
				
				candidatesOfSize.addAll(list);
			}
			
			//candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
			
			candidates.addAll(candidatesOfSize);
		}
			
		return candidates;
		
	}
	
	private double[][][][] clone4DArray(double [][][][] source){
		double[][][][] dest = new double[source.length][][][];
		for (int i = 0; i < dest.length; i++) {
			dest[i] = new double[source[i].length][][];
			for (int j = 0; j < dest[i].length; j++) {
				dest[i][j] = new double[source[i][j].length][];
				for (int k = 0; k < dest[i][j].length; k++) {
					dest[i][j][k] = new double[source[i][j][k].length];
					for (int k2 = 0; k2 < source[i][j][k].length; k2++) {
						dest[i][j][k][k2] = source[i][j][k][k2];
					}
				}
			}
		}
		return dest;		
	}
	
	private List<ISubtrajectory> fastMoveletsDiscoveryUsingCache(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize){
				
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		int n = trajectory.getData().size();
		maxSize = (maxSize == -1) ? n : maxSize;
		
		MyCounter.numberOfCandidates += (maxSize * (maxSize-1) / 2);
		/* It starts with the base case
		 * */		
		int size = 1;
				
		double[][][][] base = getBaseCase(trajectory, train);
		boolean[][][][] baseUncomparedDim = getBaseCaseUncomparedDim(base);
		
		if( size >= minSize ) candidates.addAll(getCandidatesUsingMDist(trajectory, train, size, base));
		
		double[][][][] lastSize = clone4DArray(base);
		
		/* Tratar o resto dos tamanhos 
		 * */
		for (size = 2; size <= maxSize; size++) {
	
			/* Precompute de distance matrix
			 * */
			double[][][][] newSize = getNewSize(trajectory, train, base, lastSize, size, baseUncomparedDim);
			
			/* Create candidates and compute min distances
			 * */			
			List<ISubtrajectory> candidatesOfSize = getCandidatesUsingMDist(trajectory, train, size, newSize);
			
			if (size >= minSize){
				
				for (ISubtrajectory candidate : candidatesOfSize) {
					AssesQuality(candidate);
				}
				
				candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
				
				candidates.addAll(candidatesOfSize);
			}
		
			lastSize = newSize;
						
		} // for (int size = 2; size <= max; size++)	
	
		base =  null;
		lastSize = null;
		
		candidates = MoveletsFilterAndRanker.getShapelets(candidates);
		
		return candidates;
	}


}
