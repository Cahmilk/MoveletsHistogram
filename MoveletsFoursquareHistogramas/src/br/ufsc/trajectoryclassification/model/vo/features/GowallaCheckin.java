package br.ufsc.trajectoryclassification.model.vo.features;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class GowallaCheckin implements IFeature<GowallaCheckin> {
			
	private String value;
	
	public GowallaCheckin(String value) {
		this.value = value;		
	}
	
	public String getValue() {
		return value;
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return value;
	}

	@Override
	public double getDistanceTo(GowallaCheckin other, FeatureComparisonDesc featureComparisonDesc) {
		
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		// Lowest common antecesor
		case "w2v": return getWord2VecDistance(this.value, other.getValue());
		
		case "equals": return this.value.equals(other.getValue()) ? 0 : 1 ;			
		
		// equalsIgnoreCase
		case "equalsignoreccase": return this.value.equalsIgnoreCase(other.getValue()) ? 0 : 1 ;			
		
		default:
			break;
		}
		
		// TODO Auto-generated method stub
		return -1;
	}
	
	
	/*
	 * Nesta função assumimos que tanto value1 quanto value2 existem
	 * dentro da lista vocabulary do Gowalla e 
	 * a distancia entre elas esta precalculada em W2vDistances
	 * */
	public double getWord2VecDistance(String value1, String value2){	
		
		try{
		return W2vDistances[vocabulary.get(value1)][vocabulary.get(value2)];
		} catch (Exception e) {
			// TODO: handle exception
			System.out.println(value1 + " <-> " + value2);
			return 1;
		}
		
	}

	public static Map<String, Integer> vocabulary = new HashMap<>();
	
	public static double[][] W2vDistances = null;
	
	public static void loadVocabulary(String filename){
		
		String line;
						
		int i = 0;

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
			
			/* The first line is not needed
			 * */
			//bufferedReader.readLine();
			
			while ((line = bufferedReader.readLine()) != null) {
				GowallaCheckin.vocabulary.put(line.replaceAll("\"", ""), i);				
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
		
		//System.out.println(GowallaCheckin.vocabulary.size());		
	}
	
	public static void loadW2vDistances(String filename){
		
		int size = GowallaCheckin.vocabulary.size();
		
		if (size == 0) return; 
		
		String line;
		String[] columns;
		
		GowallaCheckin.W2vDistances = new double[size][];
						
		int i = 0;

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
									
			while ((line = bufferedReader.readLine()) != null) {
				
				columns = line.split(",");				
				
				double[] doubleValues = Arrays.stream(columns)
                        .mapToDouble(Double::parseDouble)
                        .toArray();
			
				GowallaCheckin.W2vDistances[i] = doubleValues;
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
			    Arrays.stream(GowallaCheckin.W2vDistances)
			        .map(a -> Arrays.toString(a) )
			            .collect(Collectors.joining("\n"))
			);
		*/
		//System.out.println(Arrays.deepToString(GowallaCheckin.W2vDistances) );	
	}

			
}
