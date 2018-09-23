package br.ufsc.trajectoryclassification.view.similarity;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
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
import br.ufsc.trajectoryclassification.model.bo.similarity.SimilarityAnalysisMultithread;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.utils.Utils;


public class SimilarityAnalysisFoursquare {

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
				System.err.println("Parâmetro " + key + " inválido.");
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

			System.err.println("Medida de similaridade inválida.");
			
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
		 * STEP 1. Configura parâmetros de entrada
		 */
		configure(args);		
		System.out.println(showConfiguration());

		if (DESCRIPTION_FILE == null) return;
		
		String DESCRIPTION_FILE_NAME = FilenameUtils.removeExtension(
				new File(DESCRIPTION_FILE).getName());


		String resultDirPath = RESULT_DIR + "/Similarity/" + DESCRIPTION_FILE_NAME + "/" + similarityMeasure + "/";

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
		
		double[] vTime = {30, 60, 90};
		double[] vDay = {0};
		double[] vPOI = {0};
		double[] vPrice = {0,1,2};
		double[] vRating = {0.5, 1.0, 1.5};
		double[] vWeather = {0};
		
		
		
		List<double[]> thresholds = new ArrayList<double[]>();
		
		for (int iTime = 0; iTime < vTime.length; iTime++)
			for (int iDay = 0; iDay < vDay.length; iDay++)
				for (int iPOI = 0; iPOI < vPOI.length; iPOI++)
					for (int iPrice = 0; iPrice < vPrice.length; iPrice++)
						for (int iRating = 0; iRating < vRating.length; iRating++)
							for (int iWeather = 0; iWeather < vWeather.length; iWeather++){
								double[] configuration = {
										vTime[iTime],
										vDay[iDay],
										vPOI[iPOI],
										vPrice[iPrice],
										vRating[iRating],
										vWeather[iWeather]};
								thresholds.add(configuration);
										
								}
										
		
		Integer K = train.size() - 1;
		Integer kClass = 5;
		
		SimilarityAnalysisMultithread dma = new SimilarityAnalysisMultithread(train,test,dmbt,K,kClass,nthreads,resultDirPath);
		dma.setListOfThresholds(thresholds);		
		List<String> results = dma.run();
		
		for (String string : results) {
			System.out.println(string);
		}
		
		try {
			FileUtils.writeLines(new File(resultDirPath + "/similarityAnalysis.txt"), results);
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
			
	}


}
