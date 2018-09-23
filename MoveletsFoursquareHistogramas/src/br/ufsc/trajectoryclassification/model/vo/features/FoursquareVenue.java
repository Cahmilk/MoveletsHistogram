package br.ufsc.trajectoryclassification.model.vo.features;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class FoursquareVenue implements IFeature<FoursquareVenue> {
			
	private String value;
	
	public FoursquareVenue(String value) {
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
	public double getDistanceTo(FoursquareVenue other, FeatureComparisonDesc featureComparisonDesc) {
		
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		// Lowest common antecesor
		case "lca": return getLCA(this.value, other.getValue());
		
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
	 * dentro da lista venuesColumns do foursquare e 
	 * a distancia entre elas esta precalculada em venuesDistances
	 * */
	public double getLCA(String value1, String value2){	
		
		try{
		return venuesDistances[venuesColumns.get(value1)][venuesColumns.get(value2)];
		} catch (Exception e) {
			// TODO: handle exception
			System.out.println(value1 + " <-> " + value2);
			return 1;
		}
		
	}

	public static Map<String, Integer> venuesColumns = new HashMap<>();
	
	public static double[][] venuesDistances = null;
	
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

			
}
