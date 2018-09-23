package br.ufsc.trajectoryclassification.view.movelets.run;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ComparisonFeatureParameterEstimation;
import br.ufsc.trajectoryclassification.model.bo.dmbs.DMS;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.TrajectoryFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsMultithread;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.InformationGain;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCV;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureOld;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.utils.Utils;

public class MoveletsCVRun {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static int nthreads = 1;
	private static int minSize = 2;
	private static int maxSize = -1; // unlimited maxSize
	private static String strQualityMeasure = "LSP"; 	
	private static boolean cache = true;
	private static boolean exploreDimensions = false;
	private static int samples = 1;
	private static double sampleSize = 1;
	private static String medium = "none"; // Other values minmax, sd, interquartil
	private static String output = "numeric"; // Other values numeric discretized
	private static int moveletsPerTrajectory = -1; // Filtering
	
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
			case "-ms":
				minSize = Integer.valueOf(value);
				break;
			case "-Ms":
				maxSize = Integer.valueOf(value);
				break;
			case "-q":
				strQualityMeasure = value;
				break;
			case "-cache":
				cache = Boolean.valueOf(value);				
				break;
			case "-ed":
				exploreDimensions = Boolean.valueOf(value);				
				break;					
			case "-samples":
				samples = Integer.valueOf(value);				
				break;					
			case "-sampleSize":
				sampleSize = Double.valueOf(value);				
				break;					
			case "-medium":
				medium = value;				
				break;			
			case "-output":
				output = value;				
				break;	
			case "-mpt":
				moveletsPerTrajectory = Integer.valueOf(value);				
				break;					

			default:
				System.err.println("Parâmetro " + key + " inválido.");
				System.exit(1);
				return;
			}
		}

	}

	public static String showConfiguration() {

		String str = new String();

		str += "Starting running shapelets extractor " + System.getProperty("line.separator");

		str += "Configurations:" + System.getProperty("line.separator");

		str += "\tDatasets directory:	    " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:    " + RESULT_DIR + System.getProperty("line.separator");
		
		str += "\tDescription file :    " + DESCRIPTION_FILE + System.getProperty("line.separator");

		str += "\tAllowed Threads:      " + nthreads + System.getProperty("line.separator");

		str += "\tMin size:             " + minSize + System.getProperty("line.separator");

		str += "\tMax size:             " + maxSize + System.getProperty("line.separator");

		str += "\tQuality Measure:      " + strQualityMeasure + System.getProperty("line.separator");
		
		str += "\tExplore dimensions:   " + exploreDimensions + System.getProperty("line.separator");

		str += "\tSamples:      " + samples + System.getProperty("line.separator");
		
		str += "\tSample Size:   " + sampleSize + System.getProperty("line.separator");
		
		str += "\tMedium:   " + medium + System.getProperty("line.separator");
		
		str += "\tMPT:   " + moveletsPerTrajectory + System.getProperty("line.separator");
		
		str += "\tOutput:   " + output + System.getProperty("line.separator");
		
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
		
		if (exploreDimensions)
			DESCRIPTION_FILE_NAME += "_ED"; 

		String resultDirPath = RESULT_DIR + "/Movelets/" + DESCRIPTION_FILE_NAME + "/";

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
		//Utils.writeTrajectoriesToGSON(train, description, resultDirPath + "trainAfterFeatureExtraction.json");
	
		if (description.getSubtrajectoryComparisonDesc() == null){
			Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
		}
		
		List<ITrajectory> test = Utils.loadTrajectories(testDirPath, description);
				
		if (!test.isEmpty()){			
			PointFeaturesExtraction.fillAllTrajectories(test, description);		
			TrajectoryFeaturesExtraction.fillAllTrajectories(test,description);
			//Utils.writeTrajectoriesToGSON(test, description, resultDirPath + "testAfterFeatureExtraction.json");
			
			if (description.getSubtrajectoryComparisonDesc() == null){
				Utils.writeAttributesCSV(test, resultDirPath + "test.csv");
				return;
			}
		}
		
		IDistanceMeasureForSubtrajectory dms =  DMS.getDMSfromDescription(description);
		
		int folds = 3;
		
		List<List<Integer>> partitions = Utils.createIndexFoldsCV(train.size(), folds);
		
		
		for (int i = 0; i < folds; i++) {

			MyCounter.data.clear();
			
			List<ITrajectory> foldTrain = Utils.getTrainFold(train,partitions.get(i));
			List<ITrajectory> foldTest = Utils.getTestFold(train,partitions.get(i));
			String foldResultDirPath = resultDirPath + "/fold" + i + "/";
			
			for (ITrajectory item : foldTrain) {
				item.getFeatures().clear();
				item.getAttributes().clear();
			}
			
			for (ITrajectory item : foldTest) {
				item.getFeatures().clear();
				item.getAttributes().clear();
			}
			IQualityMeasure qualityMeasure = new LeftSidePureCV(foldTrain,samples,sampleSize,medium);
			
			MoveletsMultithread analysis = new MoveletsMultithread(
					foldTrain, foldTrain, foldTest, dms, minSize, nthreads, qualityMeasure, cache, exploreDimensions, medium, output, moveletsPerTrajectory, foldResultDirPath);
	     					
			analysis.run();
			
			MyCounter.data.put("candidates", MyCounter.numberOfCandidates);
			
			System.out.println(MyCounter.data);
		}
						
		
			
	}

}
