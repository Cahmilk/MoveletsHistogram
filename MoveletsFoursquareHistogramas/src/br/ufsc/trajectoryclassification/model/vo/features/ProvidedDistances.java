package br.ufsc.trajectoryclassification.model.vo.features;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class ProvidedDistances {
	
	private Map<String, Integer> vocabulary = null;
	
	private double[][] distances = null;
	
	private String vocabularyPath = null;
	
	private String distancesPath = null;
	
	
	public ProvidedDistances(String vocabularyPath, String distancesPath) {
		super();
		this.vocabularyPath = vocabularyPath;
		this.distancesPath = distancesPath;
	}
	
	public void load(){
		
		loadVocabularyFromFile(vocabularyPath);
		loadDistancesFromFile(distancesPath);
		
	} 

	
	public void loadVocabularyFromFile(String filename){
		
		String line;
		
		int i = 0;

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
			
			/* The first line is not needed
			 * */
			//bufferedReader.readLine();
			
			vocabulary = new HashMap<>();
			
			while ((line = bufferedReader.readLine()) != null) {
				vocabulary.put(line.replaceAll("\"", ""), i);				
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
	}
	
	
	public void loadDistancesFromFile(String filename){
		
		int size = vocabulary.size();
		
		if (size == 0) return; 
		
		String line;
		String[] columns;
		
		distances = new double[size][];
						
		int i = 0;

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
									
			while ((line = bufferedReader.readLine()) != null) {
				
				columns = line.split(",");				
				
				double[] doubleValues = Arrays.stream(columns)
                        .mapToDouble(Double::parseDouble)
                        .toArray();
			
				distances[i] = doubleValues;
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
		//System.out.println(Arrays.deepToString(FoursquareVenue.venuesDistances) );	
	}
	
	
	public double getDistance(String value1, String value2){
		
		try{
			int index1 = vocabulary.get(value1);
			int index2 = vocabulary.get(value2);
			/* The use of min and max allows to use only the lower triangle of the distance matrix
			 * */
			return distances[Math.min(index1, index2)][Math.max(index1, index2)];
			} catch (Exception e) {
				// TODO: handle exception
				System.out.println("Something wrong with \"distances[vocabulary.get(value1)][vocabulary.get(value2)]\" for " + value1 + " and " + value2);
				return 1;
			}		
		
	}

	
	public Map<String, Integer> getVocabulary() {
		return vocabulary;
	}

	public void setVocabulary(Map<String, Integer> vocabulary) {
		this.vocabulary = vocabulary;
	}

	public double[][] getDistances() {
		return distances;
	}

	public void setDistances(double[][] distances) {
		this.distances = distances;
	}

	public String getVocabularyPath() {
		return vocabularyPath;
	}

	public void setVocabularyPath(String vocabularyPath) {
		this.vocabularyPath = vocabularyPath;
	}

	public String getDistancesPath() {
		return distancesPath;
	}

	public void setDistancesPath(String distancesPath) {
		this.distancesPath = distancesPath;
	}
	
	
}
