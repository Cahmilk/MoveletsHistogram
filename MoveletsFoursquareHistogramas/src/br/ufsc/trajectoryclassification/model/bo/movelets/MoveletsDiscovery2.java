package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.apache.commons.collections4.ListUtils;
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


public class MoveletsDiscovery2 implements Callable<Integer> {
	
	private List<ISubtrajectory> candidates;

	private ITrajectory trajectory;
	
	private List<ITrajectory> trajectories;

	private IDistanceMeasureForSubtrajectory dmbt;
	
	private IQualityMeasure qualityMeasure;
	
	private int minSize;
	
	private int maxSize;
	
	private boolean cache;
	
	private boolean exploreDimensions;
	
	private int nthreads;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
	
	
			
	public MoveletsDiscovery2(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions, int nthreads) {
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
		this.nthreads = nthreads;
	}

	@Override
	public Integer call() throws Exception {		
	
		measureShapeletCollection();
	
		return 0;

	}

	public void measureShapeletCollection() {
		
		List<ISubtrajectory> candidates;
		
		candidates = fastMoveletsDiscoveryUsingCache(trajectory, trajectories, minSize, maxSize);
		
		/* STEP 3: SELECTING BEST CANDIDATES
		 * */		
		this.candidates.addAll(candidates);
			
	}
	
	
	private List<ISubtrajectory> getCandidatesUsingMDistParallel(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist, int nthreads) {
				
		ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(nthreads);

		executor.setRejectedExecutionHandler(new ThreadPoolExecutor.DiscardPolicy());

		List<Future<List<ISubtrajectory>>> resultList = new ArrayList<>();
		
		int n = trajectory.getData().size();
		int npositions = n - size + 1;

		int groupSize = (npositions > nthreads) ? Math.floorDiv(npositions, nthreads) + 1 : 1;
		
		List<List<Integer>> groups = ListUtils.partition(IntStream.range(0, npositions).boxed().collect(Collectors.toList()), groupSize);

		for (int i = 0; i < groups.size(); i++) {
			
			List<Integer> group = groups.get(i);
			
			if (group.isEmpty())				
				break;

			ParallelGetSubtrajectories pgs = new ParallelGetSubtrajectories(trajectory, train, size, mdist, group, dmbt, rankingAlgorithm, exploreDimensions, 2, qualityMeasure);
			
			Future<List<ISubtrajectory>> result = executor.submit(pgs);

			resultList.add(result);
		}

		/*
		 * Execute threads
		 */
		List<List<ISubtrajectory>> results = new ArrayList<>();

		for (Future<List<ISubtrajectory>> future : resultList) {

			try {

				results.add(future.get());
				
				Executors.newCachedThreadPool();

			} catch (InterruptedException | ExecutionException e) {
				e.printStackTrace();
			}
		}

		executor.shutdown();
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		for (List<ISubtrajectory> result : results) {
			candidates.addAll(result);
		}
						
		return candidates;	
	}
	
	private List<ISubtrajectory> getCandidatesUsingMDist(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist) {
		
		return getCandidatesUsingMDistParallel(trajectory, train, size, mdist, this.nthreads);
		
	}			
	
	public int getArrayIndex(double[] arr,double value) {
	    for(int i=0;i<arr.length;i++)
	        if(arr[i]==value) return i;
	    return -1;
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


	private double[][][][] getNewSize(ITrajectory trajectory, List<ITrajectory> train, double[][][][] base, double[][][][] lastSize, int size) {
		
		int n = trajectory.getData().size();	
		
		for (int start = 0; start <= (n - size); start++) {
						
			for (int i = 0; i < train.size(); i++) {
				
				if (train.get(i).getData().size() >= size) {						
							
					for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
												
						for (int k = 0; k < lastSize[start][i].length; k++) {
							
							if (lastSize[start][i][k][j] != Double.MAX_VALUE)
								
								lastSize[start][i][k][j] += base[start+size-1][i][k][j+size-1];
						
						} // for (int k = 0; k < distance.length; k++) {
											
					} // for (int j = 0; j <= (train.size()-size); j++)
					
				} // if (train.get(i).getData().size() >= size) 
				
			} // for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)
		
		return lastSize;
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
		if( size >= minSize ) candidates.addAll(getCandidatesUsingMDist(trajectory, train, size, base));
		
		double[][][][] lastSize = clone4DArray(base);
		
		/* Tratar o resto dos tamanhos 
		 * */
		for (size = 2; size <= maxSize; size++) {

			/* Precompute de distance matrix
			 * */
			double[][][][] newSize = getNewSize(trajectory, train, base, lastSize, size);
			
			/* Create candidates and compute min distances
			 * */			
			List<ISubtrajectory> candidatesOfSize = getCandidatesUsingMDist(trajectory, train, size, newSize);
						
			candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
			
			candidates.addAll(candidatesOfSize);
			
			lastSize = newSize;
						
		} // for (int size = 2; size <= max; size++)	
	
		base =  null;
		lastSize = null;
		
		candidates = MoveletsFilterAndRanker.getShapelets(candidates);
		
		return candidates;
	}


}
