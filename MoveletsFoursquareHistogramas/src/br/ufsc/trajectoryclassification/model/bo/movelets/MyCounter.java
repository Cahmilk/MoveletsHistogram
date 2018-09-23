package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.HashMap;
import java.util.Map;

public class MyCounter {

	public static long numberOfDBP = 0;
	
	public static long numberOfSubtrajectoryComparisons = 0;
	
	public static long numberOfShapelets = 0;
	
	public static long numberOfSums = 0;
	
	public static long numberOfCandidates = 0;
	
	public static long numberOfWorseCandidates = 0;
	
	public static double maxQuality = 0;
	
	public static Map<String,Long> data = new HashMap<>();
	
	public static void clear(){
		
		numberOfCandidates = 0;
		numberOfWorseCandidates = 0;
		numberOfShapelets = 0;
		
	}

	public static void addValue(String key, long value){
		
		if (data.containsKey(key)){
			Long current = data.get(key);
			data.put(key, current + value);
			}
		else
			data.put(key, value);
	} 
	
	
	
}
