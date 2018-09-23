package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.stream.Collectors;

import org.apache.commons.io.FileUtils;
import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbt.DTWA;
import br.ufsc.trajectoryclassification.model.bo.dmbt.IDistanceMeasureBetweenTrajectories;
import br.ufsc.trajectoryclassification.model.dao.ITrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.utils.ArrayIndexComparator;
import br.ufsc.trajectoryclassification.utils.ProgressBar;

public class SimilarityAnalysisMultithread  {

	private IDistanceMeasureBetweenTrajectories dmbt;							
	
	private List<ITrajectory> test;
	
	private List<ITrajectory> train;
	
	private Integer K = 1;
	
	private Integer kClass = 1;
	
	private Integer nthreads = 1;
	
	private String resultDirPath;
	
	private List<double[]> listOfThresholds;
	
	private List<double[]> listOfWeights;
	
	private List<List<String>> plainResults;
	
	
	public SimilarityAnalysisMultithread(List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureBetweenTrajectories dmbt, Integer k, Integer kClass, Integer nthreads, String resultDirPath) {
		super();		
		this.train = train;
		this.test = test;
		this.dmbt = dmbt;
		this.K = k;
		this.kClass = kClass;
		this.nthreads = nthreads;
		this.resultDirPath = resultDirPath;
	}
	
	public void setListOfThresholds(List<double[]> listOfThresholds) {
		this.listOfThresholds = listOfThresholds;
	}
	
	public void setListOfWeights(List<double[]> listOfWeights) {
		this.listOfWeights = listOfWeights;
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
	
	
	public List<String> run(){
		
		List<String> results = new ArrayList<String>();
		
		double[] weights = new double[listOfThresholds.get(0).length];
		Arrays.fill(weights, 1);
		
		ProgressBar pb = new ProgressBar();
		int step = 0;
		pb.update(step, listOfThresholds.size());
		
		for (double[] ds : listOfThresholds) {		
			
			dmbt.setThresholds(ds);
			dmbt.setWeights(weights);
			
			// Esta linha eh exclusiva para o DTWA
			if (dmbt.getClass() == DTWA.class) 	{
				System.out.println("Learning parameter S for ds = " + Arrays.toString(ds));
				((DTWA) dmbt).learnParameterS(train);
				((DTWA) dmbt).setNThreads(this.nthreads);
				System.out.println("Value: " + ((DTWA) dmbt).getFittedS());
				System.out.println("Done.");
			
			}
			
			List<TrajectoryClassificationResult> result = similarityAnalysisMultithread(train, test, dmbt, K, nthreads);
			SimilarityAnalysisEvaluation sav = new SimilarityAnalysisEvaluation(result);
			
			String str = new String();
			str += "\"" + Arrays.toString(ds) + "\"";
			str += ", " + sav.getAccuracy() + ", " + sav.getAccuracyTopKClass(this.kClass);
			System.out.println(str);
			results.add(str);
			
			String fileName = resultDirPath + Arrays.toString(ds) + ".txt";
			try {
				FileUtils.writeLines(new File(fileName), sav.getPlainResult());
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			pb.update(++step, listOfThresholds.size());
			
		}
		
		return results;
				
		
	}
	
		

}
