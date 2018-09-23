package br.ufsc.trajectoryclassification.view.movelets.run;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ComparisonFeatureParameterEstimation;
import br.ufsc.trajectoryclassification.model.bo.dmbs.DMS;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.TrajectoryFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsMultithread;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsMultithread2;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.InformationGain;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCV;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCVLigth;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureOld;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.Unsupervised1CV;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.FoursquareVenue;
import br.ufsc.trajectoryclassification.utils.Utils;

public class MoveletsRun {

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
	private static boolean lowMemory = false;
	
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
			case "-lowm":
				lowMemory = Boolean.valueOf(value);				
				break;			

			default:
				System.err.println("Parâmetro " + key + " inválido.");
				System.exit(1);
				return;
			}
		}

	}
	
	public static void loadVenuesColumns(String filename){
		
		String line;
						
		int i = 0;

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
			
			/* The first line is not needed
			 * */
			bufferedReader.readLine();
			
			while ((line = bufferedReader.readLine()) != null) {
				FoursquareVenue.venuesColumns.put(line.replaceAll("\"", ""), i);				
				i++;				
			}

			bufferedReader.close();

		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
		
		System.out.println(FoursquareVenue.venuesColumns.size());		
	}
	
	public static void loadVenuesDistances(String filename){
		
		int size = FoursquareVenue.venuesColumns.size();
		
		if (size == 0) return; 
		
		String line;
		String[] columns;
		
		FoursquareVenue.venuesDistances = new double[size][];
						
		int i = 0;

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
									
			while ((line = bufferedReader.readLine()) != null) {
				
				columns = line.split(",");				
				
				double[] doubleValues = Arrays.stream(columns)
                        .mapToDouble(Double::parseDouble)
                        .toArray();
			
				FoursquareVenue.venuesDistances[i] = doubleValues;
				i++;				
			}

			bufferedReader.close();

		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
		/*
		System.out.println(
			    Arrays.stream(FoursquareVenue.venuesDistances)
			        .map(a -> Arrays.toString(a) )
			            .collect(Collectors.joining("\n"))
			);
		*/
		System.out.println(Arrays.deepToString(FoursquareVenue.venuesDistances) );	
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
			
		List<ITrajectory> trainForMovelets = train;
		
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
		
		
		ComparisonFeatureParameterEstimation cfpe = new ComparisonFeatureParameterEstimation(train, description);
		cfpe.setFactor(1);
		cfpe.estimateParmeters();
		
		IDistanceMeasureForSubtrajectory dms =  DMS.getDMSfromDescription(description);
		
		IQualityMeasure qualityMeasure = new LeftSidePureCVLigth(trainForMovelets,samples,sampleSize,medium);
		
			
		if (lowMemory){
			
			MoveletsMultithread2 analysis = new MoveletsMultithread2(
					trainForMovelets, train, test, dms, minSize, nthreads, qualityMeasure, cache, exploreDimensions, medium, output, moveletsPerTrajectory, resultDirPath);
	     					
			analysis.run();
			
		} else {
			
			MoveletsMultithread analysis = new MoveletsMultithread(
					trainForMovelets, train, test, dms, minSize, nthreads, qualityMeasure, cache, exploreDimensions, medium, output, moveletsPerTrajectory, resultDirPath);
	     	analysis.setMaxSize(maxSize);				
			analysis.run();
			
		}
		
		
		
		MyCounter.data.put("candidates", MyCounter.numberOfCandidates);
		
		System.out.println(MyCounter.data);
			
	}

}
