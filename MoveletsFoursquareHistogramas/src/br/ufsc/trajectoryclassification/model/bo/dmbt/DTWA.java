package br.ufsc.trajectoryclassification.model.bo.dmbt;


import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.bo.similarity.SimilarityAnalysisMultithread;
import br.ufsc.trajectoryclassification.model.bo.similarity.SimilarityAnalysisThread;
import br.ufsc.trajectoryclassification.model.bo.similarity.TrajectoryClassificationResult;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.utils.ArrayIndexComparator;
import br.ufsc.trajectoryclassification.utils.ProgressBar;
import weka.attributeSelection.InfoGainAttributeEval;
import weka.classifiers.trees.j48.BinC45Split;
import weka.core.Attribute;
import weka.core.DenseInstance;
import weka.core.Instances;


public class DTWA implements IDistanceMeasureBetweenTrajectories {

	private IDistanceMeasureBetweenPoints dmbp;
	
	private double[] thresholds;
	
	private double[] weights;
	
	private double fittedS = 1.0;
	
	private int nthreads = 1;

	public double getFittedS() {
		return fittedS;
	}

	public DTWA(IDistanceMeasureBetweenPoints dmbp) {
		super();
		this.dmbp = dmbp;
	}
	

	@Override
	public double getDistance(ITrajectory t1, ITrajectory t2) {
								
		MDDTW dtwd = new MDDTW(dmbp);
		dtwd.setThresholds(this.thresholds);
		dtwd.setWeights(this.weights);
		
		DTWI dtwi = new DTWI(dmbp);
		dtwi.setThresholds(this.thresholds);
		dtwi.setWeights(this.weights);
		
		double distanceDtwd = dtwd.getDistance(t1, t2);
		double distanceDtwi = dtwi.getDistance(t1, t2);
		
		double S = (distanceDtwi / distanceDtwd);
		
		return (S < fittedS) ? distanceDtwi : distanceDtwd;

	}
	
	public void learnParameterS(List<ITrajectory> dataset){
		
		Map<String, List<Double>> scores = findScores(dataset, this.nthreads);
		
		List<Double> S_iSuccess = scores.get("S_iSuccess");
		List<Double> S_dSuccess = scores.get("S_dSuccess");
		
		double S = 1;
		
		if (S_iSuccess.isEmpty() && S_dSuccess.isEmpty())
			S = 1;
		else if (S_iSuccess.isEmpty() && !S_dSuccess.isEmpty())
			S = Collections.max(S_dSuccess);
		else if (!S_iSuccess.isEmpty() && S_dSuccess.isEmpty())
			S = Collections.max(S_dSuccess);
		else // if (!S_iSuccess.isEmpty() && !S_dSuccess.isEmpty())
			S = MaxInformationGain(S_iSuccess,S_dSuccess);
		
		this.fittedS = S;
	}

	private double MaxInformationGain(List<Double> s_iSuccess, List<Double> s_dSuccess) {
		
		ArrayList<Attribute> atts = new ArrayList<Attribute>(2);
        
        atts.add(new Attribute("distance"));
        atts.add(new Attribute("class"));
        
		Instances dataRaw = new Instances("Dataset",atts,0);
        
        for (int i = 0; i < s_iSuccess.size(); i++) {
        	double[] instanceValue1 = new double[2];
            instanceValue1[0] = s_iSuccess.get(i);
            instanceValue1[1] = 0.0; // Classe s_iSuccess        
            dataRaw.add( new DenseInstance(1.0, instanceValue1) );
		}
        
        for (int i = 0; i < s_dSuccess.size(); i++) {
        	double[] instanceValue1 = new double[2];
            instanceValue1[0] = s_dSuccess.get(i);
            instanceValue1[1] = 1.0; // Classe s_dSuccess        
            dataRaw.add( new DenseInstance(1.0, instanceValue1) );
		}
         
        BinC45Split binC45Split = new BinC45Split(0, 1, dataRaw.sumOfWeights(), true);
        dataRaw.setClassIndex(1);
        
        double splitpoint = -1;
        
        try {
			binC45Split.buildClassifier(dataRaw);
			splitpoint = binC45Split.splitPoint();			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        
        if (splitpoint == -1)
        	System.err.println("Problem to find the parameter value S for adaptive DTW.");
		// TODO Auto-generated method stub
		return splitpoint;
	}


	
	public List<TrajectoryClassificationResult> similarityAnalysisMultithread(List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureBetweenTrajectories dmbt, Integer K, Integer nthreads){
		
		List<List<ITrajectory>> groups = new ArrayList<>(nthreads);
		for (int i = 0; i < nthreads; i++) {
			groups.add(new ArrayList<ITrajectory>());
		}	
		
		for (int i = 0; i < test.size(); i++) {
			groups.get(i % nthreads).add(test.get(i));
		}
				
		ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(nthreads);

		executor.setRejectedExecutionHandler(new ThreadPoolExecutor.DiscardPolicy());

		List<Future<List<TrajectoryClassificationResult>>> resultList = new ArrayList<>();

		for (List<ITrajectory> group : groups) {
			
			SimilarityAnalysisThread sat = new SimilarityAnalysisThread(train, group, dmbt, K);
			
			Future<List<TrajectoryClassificationResult>> result = executor.submit(sat);

			resultList.add(result);		
		}
		
		/*
		 * Execute threads
		 */
		List<TrajectoryClassificationResult> results = new ArrayList<>();
		
		for (Future<List<TrajectoryClassificationResult>> future : resultList) {

			try {
				
				results.addAll(future.get());				
				Executors.newCachedThreadPool();

			} catch (InterruptedException | ExecutionException e) {
				e.printStackTrace();
			}
		}

		executor.shutdown();
		
		
		return results;
	}

	
	
	
	private Map<String, List<Double>> findScores(List<ITrajectory> dataset, int nthreads) {
		
		MDDTW dtwd = new MDDTW(dmbp);
		dtwd.setThresholds(this.thresholds);
		dtwd.setWeights(this.weights);
		
		DTWI dtwi = new DTWI(dmbp);
		dtwi.setThresholds(this.thresholds);
		dtwi.setWeights(this.weights);
		
		Map<String, List<TrajectoryClassificationResult>> rsam = parallel(dataset, nthreads);
		
		List<Double> S_iSuccess = new ArrayList<>();
		List<Double> S_dSuccess = new ArrayList<>();				
		
		for (int i = 0; i < dataset.size(); i++) {
		//for (int i = 0; i < 50; i++) {
			
			ITrajectory trajectory = dataset.get(i);
			
			Pair<ITrajectory,Double> minD = rsam.get("rsamDTWd").get(i).getTopk().get(1);
			Pair<ITrajectory,Double> minI = rsam.get("rsamDTWi").get(i).getTopk().get(1);
			
			if ( trajectory.getClass().equals( minD.getFirst().getClass() ) &&
					! trajectory.getClass().equals( minI.getFirst().getClass() ) )
			
				S_dSuccess.add( minD.getSecond() / minI.getSecond() );
			
			else if ( ! trajectory.getClass().equals( minD.getFirst().getClass() ) &&
					trajectory.getClass().equals( minI.getFirst().getClass() ) )
				
				S_iSuccess.add( minI.getSecond() / minD.getSecond() );			
			
		}
		
		Map<String, List<Double>> scores = new HashMap<>();
		scores.put("S_dSuccess", S_dSuccess);
		scores.put("S_iSuccess", S_iSuccess);
		
		// TODO Auto-generated method stub
		return scores;
	}
	
	public Map<String, List<TrajectoryClassificationResult>> parallel (List<ITrajectory> dataset, int nthreads){

		MDDTW dtwd = new MDDTW(dmbp);
		dtwd.setThresholds(this.thresholds);
		dtwd.setWeights(this.weights);
		
		DTWI dtwi = new DTWI(dmbp);
		dtwi.setThresholds(this.thresholds);
		dtwi.setWeights(this.weights);

		List<TrajectoryClassificationResult> rsamDTWd = similarityAnalysisMultithread(dataset, dataset, dtwd, 2, nthreads);
		rsamDTWd.size();
		
		List<TrajectoryClassificationResult> rsamDTWi = similarityAnalysisMultithread(dataset, dataset, dtwi, 2, nthreads);
		rsamDTWi.size();

		Map<String, List<TrajectoryClassificationResult>> rsam = new HashMap<>();
		rsam.put("rsamDTWd", rsamDTWd);
		rsam.put("rsamDTWi", rsamDTWi);

		return rsam;
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
	
	public TrajectoryClassificationResult getClassificationNearestNeighbor(List<ITrajectory> train, ITrajectory test, int K, IDistanceMeasureBetweenTrajectories dmbt){
		
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
	public IDistanceMeasureBetweenPoints getDistanceMeasureBetweenPoints() {
		// TODO Auto-generated method stub
		return dmbp;
	}

	@Override
	public void setDistanceMeasureBetweenPoints(IDistanceMeasureBetweenPoints dmbp) {
		// TODO Auto-generated method stub
		this.dmbp = dmbp;
	}

	public Object clone() throws CloneNotSupportedException{
		return this;
	}

	public void setThresholds(double[] values) {
		this.thresholds = values;
	}

	public void setWeights(double[] values) {
		// TODO Auto-generated method stub
		this.weights = values;
	}

	public void setNThreads(int nthreads) {
		// TODO Auto-generated method stub
		this.nthreads = nthreads;
	}
	
	
}
