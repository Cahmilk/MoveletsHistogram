package br.ufsc.trajectoryclassification.view.similarity;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.stream.Collectors;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.bo.dmbs.DMS;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.dmbt.MDDTW;
import br.ufsc.trajectoryclassification.model.bo.dmbt.DTWA;
import br.ufsc.trajectoryclassification.model.bo.dmbt.EDR;
import br.ufsc.trajectoryclassification.model.bo.dmbt.IDistanceMeasureBetweenTrajectories;
import br.ufsc.trajectoryclassification.model.bo.dmbt.LCSS;
import br.ufsc.trajectoryclassification.model.bo.dmbt.MSM;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.TrajectoryFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsMultithread;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCV;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCVLigth;
import br.ufsc.trajectoryclassification.model.bo.similarity.SimilarityAnalysisMultithread;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.utils.Utils;


public class SimilarityAnalysisGowallaCV {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static int nthreads = 1;
	private static IDistanceMeasureBetweenTrajectories dmbt;
	private static String similarityMeasure = null;
	
	public static void configure(String[] args) {

		for (int i = 0; i < args.length; i = i + 2) {
			String key = args[i];
			String value = args[i + 1];
			switch (key) {
			
			case "-curpath":
				CURRENT_DIR = value;
				break;
			case "-respath":
				RESULT_DIR = value;
				break;
			case "-descfile":
				DESCRIPTION_FILE = value;
				break;
			case "-nt":
				nthreads = Integer.valueOf(value);
				break;
			case "-measure":
				similarityMeasure = new String(value);				
				break;	

			default:
				System.err.println("Par�metro " + key + " inv�lido.");
				System.exit(1);
				return;
			}
		}

	}
	
	public static IDistanceMeasureBetweenTrajectories fromStringToDMBT(String key, IDistanceMeasureBetweenPoints dmbp) {

		switch (key) {
		
		case "edr": {
			return new EDR(dmbp);
		}
		case "lcss": {
			return new LCSS(dmbp);
		}
		case "mddtw": {
			return new MDDTW(dmbp);
		}
		case "msm": {
			return new MSM(dmbp);
		}
		case "dtwa": {
			return new DTWA(dmbp);
		}

		default:

			System.err.println("Medida de similaridade inv�lida.");
			
			return null;
		}
	}

	public static String showConfiguration() {

		String str = new String();

		str += "Starting running similarity analysis " + System.getProperty("line.separator");

		str += "Configurations:" + System.getProperty("line.separator");

		str += "\tBase directory:	      " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:	  " + RESULT_DIR + System.getProperty("line.separator");

		str += "\tAllowed Threads:        " + nthreads + System.getProperty("line.separator");

		str += "\tSimilarity Measure:     " + similarityMeasure + System.getProperty("line.separator");
		
//		str += "\tN Steps Best Threshold: " + nStepsToBestThreshold + System.getProperty("line.separator");
				
		return str;

	}
	
	public static void main(String[] args) {

		if (args.length == 0) return;
		/*
		 * STEP 1. Configura par�metros de entrada
		 */
		configure(args);		
		System.out.println(showConfiguration());

		if (DESCRIPTION_FILE == null) return;
		
		String DESCRIPTION_FILE_NAME = FilenameUtils.removeExtension(
				new File(DESCRIPTION_FILE).getName());


		String resultDirPath = RESULT_DIR + "/Similarity/" + DESCRIPTION_FILE_NAME + "/" + similarityMeasure + "CV" + "/";

		String trainDirPath = CURRENT_DIR + "/train";
		String testDirPath = CURRENT_DIR + "/test";
						
		String descriptionPathFile = DESCRIPTION_FILE;
				
		System.out.println("\nStarting...");
				
		/* Load description file and train and test trajectories */
		Description description = new TrajectoryDAO().loadDescription(descriptionPathFile);
					
		List<ITrajectory> train = Utils.loadTrajectories(trainDirPath, description);
				
		if (train.isEmpty()) {
			System.out.println("Empty training set");
			return;
		}
		
		PointFeaturesExtraction.fillAllTrajectories(train, description);
		TrajectoryFeaturesExtraction.fillAllTrajectories(train,description);
			
		List<ITrajectory> trainForMovelets = train;
		
		if (description.getSubtrajectoryComparisonDesc() == null){
			Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
		}
	
		List<ITrajectory> test = Utils.loadTrajectories(testDirPath, description);
				
		if (!test.isEmpty()){			
			PointFeaturesExtraction.fillAllTrajectories(test, description);		
			TrajectoryFeaturesExtraction.fillAllTrajectories(test,description);
			
			if (description.getSubtrajectoryComparisonDesc() == null){
				Utils.writeAttributesCSV(test, resultDirPath + "test.csv");
				return;
			}
		}
		
		List<String> classes = train.stream().map(e -> (String) e.getLabel()).distinct().collect(Collectors.toList());	
		
		IDistanceMeasureForSubtrajectory dms =  DMS.getDMSfromDescription(description);
		
		IDistanceMeasureBetweenTrajectories dmbt = fromStringToDMBT(similarityMeasure,dms.getDMBP());
		
		double[] vSpace = {Utils.km2deg(0.1),Utils.km2deg(0.3),Utils.km2deg(0.5)};
		double[] vTime = {30, 60, 120};
		double[] vDay = {0};
		double[] vPOI = {0};		
		
		List<double[]> thresholds = new ArrayList<double[]>();
		
		for (int iSpace = 0; iSpace < vSpace.length; iSpace++)
			for (int iTime = 0; iTime < vTime.length; iTime++)
				for (int iDay = 0; iDay < vDay.length; iDay++)
					for (int iPOI = 0; iPOI < vPOI.length; iPOI++){
						double[] configuration = {
								vSpace[iSpace],
								vTime[iTime],
								vDay[iDay],
								vPOI[iPOI],
								};
						thresholds.add(configuration);									
						}
										
		Integer kClass = 5;
				
		int folds = 2;
		int repetitions = 5;
		
		List<String> allResults = new ArrayList<>();
		
		for (int repi = 0; repi < repetitions; repi++) {

			int seed = repi;
			
			List<Map<String, List<ITrajectory>>> stratified2folds = getStratified2foldData(train,0.5,seed);
			
			for (int i = 0; i < folds; i++) {
				
				List<ITrajectory> foldTrain = stratified2folds.get(i).get("train");
				List<ITrajectory> foldTest =  stratified2folds.get(i).get("test");
				
				Integer K = foldTrain.size() - 1;
				
				String foldResultDirPath = resultDirPath + "/fold" + i + " rep"+ repi + "/";
				
				System.out.println(foldTrain.size());
				System.out.println(foldTest.size());
				
				SimilarityAnalysisMultithread dma = new SimilarityAnalysisMultithread(foldTrain,foldTest,dmbt,K,kClass,nthreads,foldResultDirPath);
				
				dma.setListOfThresholds(thresholds);		
				List<String> results = dma.run();
				
			
				for (int j = 0; j < results.size(); j++) {
					String strRepAndFold = repi + ", " + i + ", ";
					results.set(j, strRepAndFold + results.get(j) );
				}
				
				try {
					FileUtils.writeLines(new File(foldResultDirPath + "/similarityAnalysis.txt"), results);
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
						
				allResults.addAll(results);
				
			}

			
		}
		
		try {
			FileUtils.writeLines(new File(resultDirPath + "/similarityAnalysis.txt"), allResults);
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
			
	}
	
	private static List<Map<String, List<ITrajectory>>> getStratified2foldData(List<ITrajectory> train, double p, int seed) {
		// TODO Auto-generated method stub
		List<String> classes = train.stream().map(e -> (String) e.getLabel()).distinct().collect(Collectors.toList());
		
		Map<String, List<ITrajectory> > grouped = train.stream()
	            .collect(Collectors.groupingBy(ITrajectory::getLabel, Collectors.toList()));
			  
		Random random = new Random(seed);
		
		List<ITrajectory> fold1 = new ArrayList<>();
		List<ITrajectory> fold2 = new ArrayList<>();
		
		for (String myclass : classes) {			
			
			List<ITrajectory> list = grouped.get(myclass);
			int n1 = (int) Math.round(list.size() * p);
			if (n1 == 0) n1 = 1;
			Collections.shuffle(list,random);
			 		
			fold1.addAll( list.subList(0, n1) );
			fold2.addAll( list.subList(n1,list.size()));
		}
		
		Map<String, List<ITrajectory>> stratified2fold1 = new HashMap<>();
		stratified2fold1.put("train", fold1);
		stratified2fold1.put("test", fold2);
		
		Map<String, List<ITrajectory>> stratified2fold2 = new HashMap<>();
		stratified2fold2.put("train", fold2);
		stratified2fold2.put("test", fold1);
				
		List<Map<String, List<ITrajectory>>> stratified2fold = new ArrayList<>();
		stratified2fold.add(stratified2fold1);
		stratified2fold.add(stratified2fold2);
		
		return  stratified2fold;
	}



}
