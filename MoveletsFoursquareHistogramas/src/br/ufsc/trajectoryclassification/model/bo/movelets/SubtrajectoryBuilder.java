package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import org.apache.commons.math3.util.Combinations;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;

public class SubtrajectoryBuilder {
	
	public static List<ISubtrajectory> build(int start, int end, ITrajectory t, int numberOfFeatures, int numberOfTrajectories, boolean exploreDimensions, int maxNumberOfFeatures){
		
		List<ISubtrajectory> list = new ArrayList<>();
		
		int currentFeatures;
		
		if (exploreDimensions){
			currentFeatures = 1;
		} else {
			currentFeatures = numberOfFeatures;
		}
				
		for (; currentFeatures <= maxNumberOfFeatures; currentFeatures++) {
			
			for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) {
				
					list.add(new Subtrajectory(start, end, t, comb, numberOfTrajectories));
				
			}
			
		}
				
		return list;
				
	}	
	
}
