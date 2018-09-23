package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;

import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbt.IDistanceMeasureBetweenTrajectories;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.utils.ArrayIndexComparator;

public class SimilarityAnalysisThread implements Callable<List<TrajectoryClassificationResult>>  {

	private IDistanceMeasureBetweenTrajectories dmbt;							
	
	private List<ITrajectory> test;
	
	private List<ITrajectory> train;
	
	private Integer K = 1;
		
	public SimilarityAnalysisThread(List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureBetweenTrajectories dmbt, int k) {
		super();
		this.train = train;
		this.test = test;
		this.dmbt = dmbt;
		this.K = k;
	}

	public IDistanceMeasureBetweenTrajectories getDmbt() {
		return dmbt;
	}

	public void setDmbt(IDistanceMeasureBetweenTrajectories dmbt) {
		this.dmbt = dmbt;
	}

	public List<ITrajectory> getTest() {
		return test;
	}

	public void setTest(List<ITrajectory> test) {
		this.test = test;
	}

	public List<ITrajectory> getTrain() {
		return train;
	}

	public void setTrain(List<ITrajectory> train) {
		this.train = train;
	}

	public int getK() {
		return K;
	}

	public void setK(int k) {
		K = k;
	}

	public double[][] getDistanceMatrix(List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureBetweenTrajectories dmbt) {

		int n = test.size();
		int m = train.size();

		double[][] data = new double[n][m];
		
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < m; j++) {
				data[i][j] = dmbt.getDistance(test.get(i), train.get(j));
			}					
		}		
		
		return data;

	}

	public double[] getDistanceVector(List<ITrajectory> train, ITrajectory test,
			IDistanceMeasureBetweenTrajectories dmbt) {
		
		int n = train.size();

		double[] data = new double[n];
				
		for (int i = 0; i < n; i++) {			
				data[i] = dmbt.getDistance(test, train.get(i));			
		}		
		
		return data;

	}

	public TrajectoryClassificationResult getClassificationNearestNeighbor(List<ITrajectory> train, ITrajectory test, int K){
		
		double [] distances = getDistanceVector(train, test, dmbt);
		
		List<Integer> topKIndexes = getKIndex(distances, K);
			
		List<Pair<ITrajectory, Double>> topK = new ArrayList<Pair<ITrajectory, Double>>();
						
		for (int k = 0; k < topKIndexes.size(); k++) {
			ITrajectory trajectory = train.get(topKIndexes.get(k));
			Double distance = distances[topKIndexes.get(k)];
			Pair<ITrajectory, Double> item = new Pair<ITrajectory, Double>(trajectory, distance);
			topK.add(item);
		}
			
		TrajectoryClassificationResult tcr = new TrajectoryClassificationResult(test, topK);
			
		return tcr;		
		
	}
		
	
	public List<Integer> getKIndex( double [] distances, int k ){		
		ArrayIndexComparator comparator = new ArrayIndexComparator(distances);
		Integer[] indexes = comparator.createIndexArray();
		Arrays.sort(indexes, comparator);
		return Arrays.asList(indexes).subList(0, k);
	}

	@Override
	public List<TrajectoryClassificationResult> call() throws Exception {
		
		List<TrajectoryClassificationResult> vtcr = new ArrayList<TrajectoryClassificationResult>();
		
		for (int i = 0; i < test.size(); i++) {
			TrajectoryClassificationResult tcr = getClassificationNearestNeighbor(train, test.get(i), K);
			vtcr.add(tcr);
		}
		
		// TODO Auto-generated method stub
		return vtcr;
	}
	
		

}
