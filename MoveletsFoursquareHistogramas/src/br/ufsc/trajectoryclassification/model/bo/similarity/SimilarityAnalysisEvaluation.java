package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Pair;

public class SimilarityAnalysisEvaluation {
	
	private List<TrajectoryClassificationResult> result;

	public SimilarityAnalysisEvaluation(List<TrajectoryClassificationResult> result) {
		super();
		this.result = result;
	}
	
	public double getAccuracy(){
		
		int correctInstances = 0;
		
		for (TrajectoryClassificationResult tcr : result) {
			if (tcr.isInTop1())
				correctInstances++;
		}
		
		double accuracy = 1.0d * correctInstances / result.size();
		
		return accuracy;		
		
	}
	
	public double getAccuracyTopK(){
		
		int correctInstances = 0;
		
		for (TrajectoryClassificationResult tcr : result) {
			if (tcr.isInTopK())
				correctInstances++;
		}
		
		double accuracy = 1.0d * correctInstances / result.size();
		
		return accuracy;		
		
	}
	
	public List<String> getPlainResult(){
		
		List<String> data = new ArrayList<>();
		
		for (TrajectoryClassificationResult tcr : result) {
			
			data.add( tcr.getTrajectory().getLabel() + ", " + tcr.getTopk().get(0).getFirst().getLabel() );
								
		}
		
		return data;
				
	} 
	
	public double getAccuracyTopKClass(int kClass){
		
		int correctInstances = 0;
		
		for (TrajectoryClassificationResult tcr : result) {
			if (tcr.isInTopKClass(kClass))
				correctInstances++;
		}
		
		double accuracy = 1.0d * correctInstances / result.size();
		
		return accuracy;		
		
	}
	
}
