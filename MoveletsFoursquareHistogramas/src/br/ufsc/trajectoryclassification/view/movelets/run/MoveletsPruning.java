package br.ufsc.trajectoryclassification.view.movelets.run;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ComparisonFeatureParameterEstimation;
import br.ufsc.trajectoryclassification.model.bo.dmbs.DMS;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.TrajectoryFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFilterAndRanker;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFinding;
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
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.features.FoursquareVenue;
import br.ufsc.trajectoryclassification.utils.SubtrajectoryGSON;
import br.ufsc.trajectoryclassification.utils.Utils;

public class MoveletsPruning {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static int nthreads = 1;
	private static String strQualityMeasure = "LSP"; 	
	private static boolean exploreDimensions = false;
	private static String attributeSelection = "Novelty1"; 
	private static String medium = "none"; // Other values minmax, sd, interquartil
	private static String output = "numeric"; // Other values numeric discretized
	
	
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
			case "-q":
				strQualityMeasure = value;
				break;
			case "-ed":
				exploreDimensions = Boolean.valueOf(value);				
				break;
			case "-output":
				output = value;				
				break;
			case "-medium":
				medium = value;				
				break;	
			case "-as":
				attributeSelection = value;
			
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

		str += "Movelets pruning " + System.getProperty("line.separator");

		str += "Configuration:" + System.getProperty("line.separator");

		str += "\tDatasets directory:	    " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:    " + RESULT_DIR + System.getProperty("line.separator");
		
		str += "\tAttribute Selection Method   " + attributeSelection + System.getProperty("line.separator");

		str += "\tAllowed Threads:      " + nthreads + System.getProperty("line.separator");
		
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
		
		String resultDirPathAS = RESULT_DIR + "/Movelets" + attributeSelection + "/" + DESCRIPTION_FILE_NAME + "/";
  
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
		
		IDistanceMeasureForSubtrajectory dmbt =  DMS.getDMSfromDescription(description);
		
		List<ISubtrajectory> prunnedMovelets = new ArrayList<>();
		
		for (String myclass : classes) {
			
			if ( (new File(resultDirPath + myclass + "/moveletsOnTrain.json").exists()) ) {
			
				System.out.println("Class: " + myclass + ". Loading movelets.");
								
				List<ISubtrajectory> movelets = Utils.readMoveletsToGSON(resultDirPath + myclass + "/moveletsOnTrain.json", train);
				System.out.print(movelets.size());
				//movelets = MoveletsFilterAndRanker.Redundance2(movelets,dmbt);
				movelets = MoveletsFilterAndRanker.noveltyFilter1(movelets,3);
				prunnedMovelets.addAll(movelets);
				
				// Train Transformation				
				movelets.forEach(e -> e.setDistances(null));
				train.forEach(e -> e.getAttributes().clear());
				new MoveletsFinding(movelets, train, dmbt).run();		
				movelets.forEach(movelet -> Utils.putAttributeIntoTrajectories(train, movelet, output, medium) );
				Utils.writeAttributesCSV(train, resultDirPathAS + myclass + "/" + "train.csv");				
				Utils.writeShapeletsToGSON(train, movelets, dmbt.getDescription(), resultDirPathAS + myclass + "/" + "moveletsOnTrain.json");	
				
				// Test Transformation				
				movelets.forEach(e -> e.setDistances(null));
				test.forEach(e -> e.getAttributes().clear());
				new MoveletsFinding(movelets, test, dmbt).run();		
				movelets.forEach(movelet -> Utils.putAttributeIntoTrajectories(test, movelet, output, medium) );
				Utils.writeAttributesCSV(test, resultDirPathAS + myclass + "/" + "test.csv");				
				Utils.writeShapeletsToGSON(test, movelets, dmbt.getDescription(), resultDirPathAS + myclass + "/" + "moveletsOnTest.json");		
				
				System.out.println(" -> " + movelets.size());
				
			} else {
				
				System.out.println("Class: " + myclass + ". Not performed yet.");
			}
			
		}
		
		
		// Train Transformation
		prunnedMovelets.forEach(e -> e.setDistances(null));
		train.forEach(e -> e.getAttributes().clear());
		new MoveletsFinding(prunnedMovelets, train, dmbt).run();		
		prunnedMovelets.forEach(movelet -> Utils.putAttributeIntoTrajectories(train, movelet, output, medium) );
		Utils.writeAttributesCSV(train, resultDirPathAS + "train.csv");				
		Utils.writeShapeletsToGSON(train, prunnedMovelets, dmbt.getDescription(), resultDirPathAS + "moveletsOnTrain.json");		

		
		// Train Transformation		
		prunnedMovelets.forEach(e -> e.setDistances(null));
		test.forEach(e -> e.getAttributes().clear());
		new MoveletsFinding(prunnedMovelets, train, dmbt).run();		
		prunnedMovelets.forEach(movelet -> Utils.putAttributeIntoTrajectories(train, movelet, output, medium) );
		Utils.writeAttributesCSV(train, resultDirPathAS + "test.csv");				
		Utils.writeShapeletsToGSON(train, prunnedMovelets, dmbt.getDescription(), resultDirPathAS + "moveletsOnTest.json");		

		
		MyCounter.data.put("candidates", MyCounter.numberOfCandidates);
		
		System.out.println(MyCounter.data);
			
	}

}
