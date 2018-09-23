package br.ufsc.trajectoryclassification.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.apache.commons.collections4.ListUtils;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.util.Pair;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.internal.LinkedTreeMap;
import com.google.gson.reflect.TypeToken;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.dmbt.IDistanceMeasureBetweenTrajectories;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Trajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import weka.core.DenseInstance;

public class Utils {

	public static final String CURRENT_DIR = "D:/Users/andres/git_projects/datasets/taxis3000/";
	
	

	public static double getMinDistance(List<Double> distances) {

		double min = Double.MAX_VALUE;

		double distance = 0;

		for (int i = 0; i < distances.size(); i++) {
			distance = distances.get(i);
			if (distance < min)
				min = distance;
		}

		return min;
	}
	
	//Just check wether one is bigger than the other
	private static boolean firstVectorGreaterThanTheSecond(double [] first, double [] second){
		
		for (int i = 0; i < first.length; i++) {
			if (first[i] <= second[i])
				return false;
		}
		
		return true;
	} 
	
	//Get the distances from one subrajectory to a trajectory
	//função horrorosa, vamos tentar enviar pra outro lugar isso aqui.
	public static double[][] getDistancesFromMoveletsForHistogram(ISubtrajectory s, ITrajectory t, IDistanceMeasureForSubtrajectory dmbt) {

		IDistanceMeasureBetweenPoints dmbp = dmbt.getDMBP();
		
		double[] maxValues = new double[dmbp.getNumberOfFeatures()];
		Arrays.fill(maxValues, Double.MAX_VALUE);
				
		if (s.getData().size() > t.getData().size())
			return null;

		List<IPoint> menor = s.getData();
		List<IPoint> maior = t.getData();
		
		int diffLength = maior.size() - menor.size();		
				
		int[] comb = s.getPointFeatures();
		double currentSum[] = new double[comb.length];
		double[] values = new double[dmbp.getNumberOfFeatures()];
		double[][] distancesForT = new double[comb.length][diffLength+1];
						
		double[] x = new double[comb.length];
		Arrays.fill(x, Double.MAX_VALUE);
				
		for (int i = 0; i <= diffLength; i++) {

			Arrays.fill(currentSum, 0);
						
			for (int j = 0; j < menor.size(); j++) {

				values = dmbp.getDistance(menor.get(j), maior.get(i + j));

				for (int k = 0; k < comb.length; k++) {					
					if (currentSum[k] != Double.MAX_VALUE && values[k] != Double.MAX_VALUE)
						currentSum[k] += values[comb[k]] * values[comb[k]];
					else {
						currentSum[k] = Double.MAX_VALUE;
					}
				}
				
				
				if (firstVectorGreaterThanTheSecond(currentSum, x) ){
					for (int k = 0; k < comb.length; k++) {
						currentSum[k] = Double.MAX_VALUE;
					}					
					break;					
				} 											
				
			}
			
			if (firstVectorGreaterThanTheSecond(x, currentSum) ){
				for (int k = 0; k < comb.length; k++) {
					x[k] = currentSum[k];					
					}				
			}
			
			for (int k = 0; k < comb.length; k++) {
				distancesForT[k][i] = currentSum[k];
			}
		}
		
		for( int i =0; i<distancesForT.length; i++) {
			
			for(int j =0; j<distancesForT[i].length; j++) {
				
				distancesForT[i][j] = (distancesForT[i][j] != Double.MAX_VALUE) ? Math.sqrt(distancesForT[i][j] / menor.size()) 
						   : Double.MAX_VALUE;
				
			}
			
		}
		
		return distancesForT;
	}	

	public static double mean(List<Double> values) {

		double sum = 0;

		for (int i = 0; i < values.size(); i++) {
			sum += values.get(i);
		}

		return sum / values.size();
	}

	public static double rad2deg(double rad) {
		return rad * 180.0 / Math.PI;
	}

	public static double km2deg(double km) {
		double R = 6371.1;
		return rad2deg(km / R);
	}

	public static double M100_IN_DEGREES = km2deg(0.1);	

	public static double[][] append(double[][] a, double[][] b) {
		if (a == null){
			double[][] result = new double[b.length][];
			System.arraycopy(b, 0, result, 0, b.length);
			return result;
		}
		else if (b == null){
			double[][] result = new double[a.length][];
			System.arraycopy(a, 0, result, 0, a.length);
			return result;
		}
		else {
			double[][] result = new double[a.length + b.length][];
			System.arraycopy(a, 0, result, 0, a.length);
			System.arraycopy(b, 0, result, a.length, b.length);
			return result;
		}
	}

	public static double normalize(double value, double max) {
		// Este método está preparado para que, se o valor max for negativo
		// Ele não realiza a normalização. Assim, estes serão os valores por
		// padrão
		// Para as variáveis de max.
		if (max < 0)
			return value;
		else if (value > max)
			return 1;
		else
			return value / max;

	}

	public static void statsMemory() {

		int mb = 1024 * 1024;

		// get Runtime instance
		Runtime instance = Runtime.getRuntime();

		System.out.println("***** Heap utilization statistics [MB] *****\n");

		// available memory
		System.out.println("Total Memory: " + instance.totalMemory() / mb);

		// free memory
		System.out.println("Free Memory: " + instance.freeMemory() / mb);

		// used memory
		System.out.println("Used Memory: " + (instance.totalMemory() - instance.freeMemory()) / mb);

		// Maximum available memory
		System.out.println("Max Memory: " + instance.maxMemory() / mb);

	}

	public static double[][] getDistanceMatrix(List<ITrajectory> trajectories,
			IDistanceMeasureBetweenTrajectories dmbt) {

		int n = trajectories.size();

		double[][] data = new double[n][n];

		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				data[i][j] = dmbt.getDistance(trajectories.get(i), trajectories.get(j));
			}
		}
		
		return data;

	}

	public static double[][] getDistanceMatrix(List<ITrajectory> train, List<ITrajectory> test,
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

	public static double estimateMaxValueFromDistanceMatrix(double[][] data) {

		double sum = 0;
		int n = Math.round(data.length / 10);
		int m = Math.round(data[0].length / 10);

		for (int i = 0; i < n; i++) {
			for (int j = 0; j < m; j++) {
				sum += data[i][j];
			}
		}

		System.out.println(sum);
		return sum / (n * m);
	}

	public static void distanceMatrixToCSV(String filename, double[][] distanceMatrix,	List<String> distanceMatrixLabels) {

		try {
			FileWriter file = new FileWriter(filename);
			BufferedWriter writer = new BufferedWriter(file);
			String header = new String();
			// Doing the header
			for (int i = 0; i < distanceMatrix[0].length; i++) {
				header += "att" + (i+1) + ",";
			}
			header += "class";
			writer.write(header + System.getProperty("line.separator"));
			
			// Doing the data
			for (int i = 0; i < distanceMatrix.length; i++) {
				double[] ds = distanceMatrix[i];
				writer.write(Arrays.toString(ds).replaceAll("[\\[|\\]|\\s]", "") + "," + "\"" + distanceMatrixLabels.get(i) + "\""
						+ System.getProperty("line.separator"));							
			}
			
			writer.flush();						
			writer.close();			

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	public static void stopIfErrorValues(Map<String,IFeature> features){
		for (String key : features.keySet()) {
			Double value = ((Numeric) features.get(key)).getValue();
			if (value.isNaN() || value.isInfinite()){
				System.out.println(key+" "+value);
				System.exit(1);
			}
		}
	}

	public static List<ITrajectory> loadTrajectories(String dirPath, Description description){
		
		String filepath = dirPath + ".zip"; 
		
		if (!new File(filepath).exists()) return new ArrayList<>();
		
		UnzipUtility.unzip(filepath, dirPath);
		
		List<ITrajectory> trajectories = new TrajectoryDAO().loadFromDir(dirPath,description);
		
		try {
			FileUtils.deleteDirectory(new File(dirPath));			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return trajectories;	
		
	}

	public static void writeShapelets(List<ISubtrajectory> shapelets, String filepath) {

		BufferedWriter writer;

		try {
			
			File file = new File(filepath);
			file.getParentFile().mkdirs();
			writer = new BufferedWriter(new FileWriter(file));

			for (ISubtrajectory subtrajectory : shapelets) {
				writer.write(subtrajectory.toString() + System.getProperty("line.separator"));
			}

			writer.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public static void putAttributeIntoTrajectories(List<ITrajectory> trajectories, ISubtrajectory shapelet, String output, String medium) {

		switch (output){
		
			case "numeric" :
				putAttributeIntoTrajectoriesNumeric(trajectories,shapelet);
				break;
			
			case "discrete" :	
				putAttributeIntoTrajectoriesDiscrete(trajectories,shapelet,medium);
				break;
				
		}
		
	}
	
	public static void putAttributeIntoTrajectoriesNumeric(List<ITrajectory> trajectories, ISubtrajectory movelet) {

		String attributeName = "sh_TID" + movelet.getTrajectory().getTid() + 
									"_START" + movelet.getStart() + 
									"_SIZE" + movelet.getSize() + 
									"_CLASS" + movelet.getTrajectory().getLabel();

		RealMatrix rm = new Array2DRowRealMatrix(movelet.getDistances());
				
		double[] splitpoints = movelet.getSplitpoints();
		double[] maxDistances = movelet.getMaxDistances();
				
		double distance;
		
		for (int i = 0; i < trajectories.size(); i++) {
			
			if (isCovered(rm.getColumn(i), splitpoints)){
				distance = normalizeCovered(rm.getColumn(i), splitpoints);
			} else {
				distance = normalizeNonCovered(rm.getColumn(i), splitpoints, maxDistances);
			}
				
			trajectories.get(i).getAttributes().put(attributeName, distance);
		}
	}

	public static void putAttributeIntoTrajectoriesDiscrete(List<ITrajectory> trajectories, ISubtrajectory movelet, String medium) {

		String attributeName = "sh_TID" + movelet.getTrajectory().getTid() + 
									"_START" + movelet.getStart() + 
									"_SIZE" + movelet.getSize() + 
									"_CLASS" + movelet.getTrajectory().getLabel();

		double[][] distances = movelet.getDistances();		
		RealMatrix rm = new Array2DRowRealMatrix(distances);
		Map<String,double[]> splitpointsData = movelet.getSplitpointData();		
		
		Pair<double[], double[]> splitpointLimits = Utils.fillSplitPointsLimits(splitpointsData,medium);
		double[] splitpointsLI = splitpointLimits.getFirst(); // Limite inferior
		double[] splitpointsLS = splitpointLimits.getSecond(); // Limite superior
        	
		for (int i = 0; i <  trajectories.size(); i++) {
			double distance;
			if (Utils.isCovered(rm.getColumn(i), splitpointsLI))
				distance = 0;
			else if (Utils.isCovered(rm.getColumn(i), splitpointsLS))
				distance = 1;
			else 
				distance = 2;
			
			trajectories.get(i).getAttributes().put(attributeName, distance);
		}		
	
	
	}
	
	
	public static Pair<double[],double[]> fillSplitPointsLimits(Map<String, double[]> splitpointsData, String medium){
		int n = splitpointsData.get("mean").length;
		double[] splitpointsLI = new double[n];
		double[] splitpointsLS = new double[n];
		
		switch (medium){
		
		case "interquartil" :
			splitpointsLI = splitpointsData.get("p25");
			splitpointsLS = splitpointsData.get("p75");				
			break;
		case "sd" :
			for (int i = 0; i < n; i++) {
				splitpointsLI[i] = splitpointsData.get("mean")[i] - splitpointsData.get("sd")[i];
				splitpointsLS[i] = splitpointsData.get("mean")[i] + splitpointsData.get("sd")[i];
			}
			break;
		case "minmax" :
			splitpointsLI = splitpointsData.get("min");
			splitpointsLS = splitpointsData.get("max");				
			break;
		case "mean" :
			splitpointsLI = splitpointsData.get("mean");
			splitpointsLS = splitpointsData.get("mean");	
			break;	
			
		default :
			splitpointsLI = splitpointsData.get("mean");
			splitpointsLS = splitpointsData.get("mean");					
	
	}		
		
		return new Pair(splitpointsLI,splitpointsLS);
	}
	
	
	private static double normalizeCovered(double[] point, double[] limits) {
		
		int dimensions = limits.length;
		double sumOfProportions = 0;
		
		for (int i = 0; i < dimensions; i++) {
			sumOfProportions += point[i] / limits[i];
		}
		
		return sumOfProportions / dimensions;
	}

	private static double normalizeNonCovered(double[] point, double[] limits, double[] maxDistances) {
		
		int dimensions = limits.length;
		double sumOfProportions = 0;
		
		if (maxDistances == null){
			maxDistances = new double[point.length];
			Arrays.fill(maxDistances, Double.MAX_VALUE);
		}
		
		for (int i = 0; i < dimensions; i++) {
			
			if (point[i] >= maxDistances[i]){
				point[i] = maxDistances[i];				
			}
							
			//sumOfProportions += point[i] / maxDistances[i];
			sumOfProportions += point[i] / limits[i];
		}
		
		return 1.0 + sumOfProportions / dimensions;
		//return sumOfProportions / dimensions;
	}

	/*
	public static boolean isCovered(double[] point, double[] limits){
		
		int dimensions = limits.length;
		
		for (int i = 0; i < dimensions; i++) {
			if (point[i] >= limits[i])
				return false;
		}
		
		return true;
	}*/
	
	/* Para o caso de empate por conta de movelets discretas
	 */
	public static boolean isCovered(double[] point, double[] limits){
		
		int dimensions = limits.length;
		
		for (int i = 0; i < dimensions; i++) {
			if (limits[i] > 0){
				if (point[i] >= limits[i])
					return false;
			} else
				if (point[i] > limits[i])
					return false;
		}
		
		return true;
	}
	
	public static void writeTrajectoriesToGSON(List<ITrajectory> trajectories, Description description, String filePath){

		try {			
			File file = new File(filePath);
			file.getParentFile().mkdirs();
			
			FileWriter fileWriter = new FileWriter(filePath);
			Gson gson = new GsonBuilder().setPrettyPrinting().create();
			gson.toJson(trajectories, fileWriter);
			fileWriter.write(System.getProperty("file.separator"));
			fileWriter.close();

			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		
	}
	
	
	public static List <ISubtrajectory> readMoveletsToGSON(String filePath, List<ITrajectory> trajectories){
		

		TOGSON fromGSON = null;
		Gson gson = new GsonBuilder().setPrettyPrinting().create();
		
		try {
			File file = new File(filePath);
			file.getParentFile().mkdirs();
			
			FileReader fileReader = new FileReader(filePath);
			
			fromGSON = gson.fromJson(fileReader, TOGSON.class);			
									
			fileReader.close();

			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.getLocalizedMessage();
			e.printStackTrace();
		}
				
		
		List<SubtrajectoryGSON> subtrajectoryGSONs = fromGSON.getShapelets();
		
		List <ISubtrajectory> movelets =  
				subtrajectoryGSONs.stream().map(e -> e.toSubtrajectory(trajectories)).collect(Collectors.toList());
	
		return movelets;
		
	}
	
	
	public static void writeShapeletsToGSON(List<ITrajectory> trajectories, List<ISubtrajectory> shapelets, Description description, String filePath){
		
		List<Map<String,Object>> classOfTrajectories = new ArrayList<>();
		
		for (ITrajectory t : trajectories) {			
			Map<String, Object> classOfT = new HashMap<>();
			classOfT.put("tid", t.getTid());
			classOfT.put("label", t.getLabel());			
			classOfTrajectories.add(classOfT);
		}
				
		List <SubtrajectoryGSON> subtrajectoryToGSONs = new ArrayList<>();
		
		for (ISubtrajectory shapelet : shapelets) {
			subtrajectoryToGSONs.add(SubtrajectoryGSON.fromSubtrajectory(shapelet,description));		
			//System.out.println(SubtrajectoryToGSON.fromSubtrajectory(shapelet).getFeatures());
		}
		
		TOGSON toGSON = new TOGSON(classOfTrajectories, subtrajectoryToGSONs);
		
		try {
			File file = new File(filePath);
			file.getParentFile().mkdirs();
			
			FileWriter fileWriter = new FileWriter(filePath);
			Gson gson = new GsonBuilder().setPrettyPrinting().create();
			
			gson.toJson(toGSON, fileWriter);
			fileWriter.close();

			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.getLocalizedMessage();
			e.printStackTrace();
		} 
		
				
	}
	
	
	public static void writeAttributesCSV(List<ITrajectory> trajectories, String filepath) {
		// TODO Auto-generated method stub
		BufferedWriter writer;

		try {
			
			File file = new File(filepath);
			file.getParentFile().mkdirs();
			writer = new BufferedWriter(new FileWriter(file));
						
			String header = (!trajectories.get(0).getFeatures().keySet().isEmpty()) ? 
					trajectories.get(0).getFeatures().keySet().toString().replaceAll("[\\[|\\]|\\s]", "") + "," : "";
				
			header += (!trajectories.get(0).getAttributes().keySet().isEmpty()) ?
					trajectories.get(0).getAttributes().keySet().toString().replaceAll("[\\[|\\]|\\s]", "") + "," : ""; 
			
			header += "class" + System.getProperty("line.separator");
			
			writer.write(header);
			
			for (ITrajectory trajectory : trajectories) {
				
				String line = (!trajectory.getFeatures().values().isEmpty()) ?
								trajectory.getFeatures().values().toString().replaceAll("[\\[|\\]|\\s]", "") + "," : "";
				
				line += (!trajectory.getAttributes().values().isEmpty()) ?
						trajectory.getAttributes().values().toString().replaceAll("[\\[|\\]|\\s]", "") + "," : "";
				
				line += "\"" + trajectory.getLabel() + "\""+ System.getProperty("line.separator");
				
				writer.write(line);
			
			}

			writer.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	private static String getSubtrajectoryHeader(String line, List<FeatureComparisonDesc> list_possible_features) {

		line = line + "MovID,";
		
		for(FeatureComparisonDesc feature : list_possible_features) 
			 line = line + feature.getText() + ",";
			
		line = line + "MovName,Label,TID";
		line = line + System.lineSeparator();
		
		return line;
	}
	
	
	private static String getPointInfo(String line, IPoint point, List<FeatureComparisonDesc> list_possible_features, ITrajectory mother_trajectory, int[] features, ISubtrajectory subtrajectory) {
				
		HashMap<String, IFeature> point_features = point.getFeatures();
		
		int k = 0;
		
		for(FeatureComparisonDesc feature:list_possible_features ) {
			
			if(ArrayUtils.contains(features, k)) 
				line = line + point.getFeature(feature.getText()) + ",";
			
			else
				line = line + "0.0,";
			
			k++;
			
		}
		
		line = line + getMoveletName(subtrajectory) + ",";
		line = line + mother_trajectory.getLabel() + ",";
		line = line + mother_trajectory.getTid();   
		line = line + System.lineSeparator();
		return line;
		
	}
	
	private static String getSubtrajectoryFeaturesByPoints(ISubtrajectory subtrajectory, int counter, IDistanceMeasureForSubtrajectory dmbt){
		
		String line = "";
		
		ITrajectory mother_trajectory = subtrajectory.getTrajectory();
		
		//Get the features present in a subtrajectory
		int[] features = subtrajectory.getPointFeatures();
		
		//Get the possible features present in a movelet.
		List<FeatureComparisonDesc> list_possible_features = dmbt.getDescription().getPointComparisonDesc().getFeatureComparisonDesc();
		
		//Get the points in a movelet.
		List<IPoint> pointsInMovelet = subtrajectory.getData();
		
		//Put in the line the movelet ID and the trajectory ID. 
		if(counter == 0 ) 
			line = getSubtrajectoryHeader(line, list_possible_features);
		
		//Each point is analyzed
		for(IPoint point : pointsInMovelet) {
			
			line = line + counter + ",";
			line = getPointInfo(line, point, list_possible_features, mother_trajectory, features, subtrajectory);
			
		}
		
		return line;
		
	}
	
	//Get each movelet name
	private static String getMoveletName(ISubtrajectory movelet) {
		
		String line = "";
		
		line = line + "sh_TID" + movelet.getTrajectory().getTid() + "_START" + movelet.getStart() + "_SIZE" + movelet.getSize() + "_CLASS" + movelet.getTrajectory().getLabel();
		
		return line;
		
	}
	
	public static void writeSubAttributesCSV(List<ISubtrajectory> subtrajectories, String filepath, IDistanceMeasureForSubtrajectory dmbt) {
		// TODO Auto-generated method stub
		BufferedWriter writer;

		try {
			
			File file = new File(filepath);
			file.getParentFile().mkdirs();
			writer = new BufferedWriter(new FileWriter(file));
			
			int counter = 0;
			
			//For each subtrajectory, or movelet, and print each line of it.
			for (ISubtrajectory subtrajectory : subtrajectories) {
				
				String line = "";
				
				line = getSubtrajectoryFeaturesByPoints(subtrajectory, counter, dmbt);
				
				writer.write(line);

				counter = counter + 1;
				
			}

			writer.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public static void dimensionalityReduction(List<ITrajectory> trajectories, double rate) {
		
		if (rate <= 0 || rate >= 1) return;
		
		// TODO Auto-generated method stub
		Random random = new Random(1);
		for (int i = 0; i < trajectories.size(); i++) {
			ITrajectory t = trajectories.get(i);
			int size = t.getData().size();
			int newSize = size - (int) Math.round(rate * size);
			
			List<Integer> list = IntStream.range(0, size).boxed().collect(Collectors.toList());
			Collections.shuffle(list,random);
			list = new ArrayList<>(list.subList(0, newSize));
			Collections.sort(list);
						
			List<IPoint> newData = new ArrayList<>();			
			for (Integer position : list) {
				newData.add(t.getData().get(position));
			}
			
			trajectories.set(i, new Trajectory(t.getTid(),newData, t.getLabel()));
		}
		
	}

	public static void selectingPoints(List<ITrajectory> trajectories, int maxPoints) {
		
		for (int i = 0; i < trajectories.size(); i++) {
			ITrajectory t = trajectories.get(i);				
			int size = t.getData().size();
			
			if (size > maxPoints){
				int fromIndex = maxPoints;
				int toIndex = size;
				t.getData().removeAll(  t.getData().subList(fromIndex, toIndex) );				
			}			
			
		}
		
	}
	
	public static List<List<Integer>> createIndexFoldsCV(int datasetSize, int folds){
		
		return createIndexFoldsCV(datasetSize, folds, 1);
		
	}
	
	public static List<List<Integer>> createIndexFoldsCV(int datasetSize, int folds, int seed){
		
		if (datasetSize < folds){
			System.err.println("Very little dataset to perform cross validations");
			System.exit(1);
		}
		
		List<Integer> integers =  IntStream.range(0, datasetSize).boxed().collect(Collectors.toList());
			
			
		Collections.shuffle(integers, new Random(seed));		
		
		int partitionSize = integers.size() / folds;
		int remaind = integers.size() % folds;		
		
		List<List<Integer>> partitions = new ArrayList<>();
		
		int fromIndex = 0;
		int toIndex = partitionSize + ((remaind > 0) ? 1 : 0);
		for (int i = 0; i < folds; i++) {
			 partitions.add(new ArrayList<>(integers.subList(fromIndex, toIndex)));
			 fromIndex = toIndex;
			 toIndex += partitionSize + ((remaind > (i+1)) ? 1 : 0);
		}
		
		return partitions;
	}
	
	public static List<ITrajectory> getTestFold(List<ITrajectory> trajectories, List<Integer> partition){
		
		List<ITrajectory> selectedTrajectories = new ArrayList<>();
		
		for (int i = 0; i < partition.size(); i++) {
			selectedTrajectories.add(trajectories.get(partition.get(i)));
		}
		
		return selectedTrajectories;
		
	}

	public static List<ITrajectory> getTrainFold(List<ITrajectory> trajectories, List<Integer> partition){
		
		List<ITrajectory> selectedTrajectories = new ArrayList<>(trajectories); 
		
		selectedTrajectories.removeAll(getTestFold(trajectories, partition));
		
		return selectedTrajectories;
		
	}
	
	public static Map<String,Integer> summaryMovelets(List<ISubtrajectory> movelets){
		
		Map<String,Integer> map = new HashMap<String,Integer>();
		
		for (ISubtrajectory m : movelets) {
			String str =  Arrays.toString( m.getPointFeatures() );
			if (map.containsKey(str))
				map.put(str, (map.get(str) + 1) );
			else 
				map.put(str, new Integer(1) );
		}
				
		return map;
	}

	
}
