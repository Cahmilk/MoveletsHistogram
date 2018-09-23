package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class ClassificationResultsTopKEval {
	
	private List<ClassificationResultTopK> data;
	
	public ClassificationResultsTopKEval(List<ClassificationResultTopK> data) {
		super();
		this.data = data;
	}


	public List<ClassificationResultTopK> getData() {
		return data;
	}
	
	public double errorRate(){	
		
		double errorRate = 0;
		
		for (ClassificationResultTopK c : data) {
			
			if ( !c.getObserved().contains(c.getExpected()) )
					
				errorRate += 1.0d;
		}
						
		return (errorRate) / (double) data.size();
		
	}
	
	public double accuracy(){	
		
		double truePositive = 0;
		
		for (ClassificationResultTopK c : data) {
			
			if ( c.getObserved().get(0).equals(c.getExpected()) )
					
				truePositive += 1.0d;
		}
						
		return (truePositive) / (double) data.size();
		
	}
	
	public double accuracyTopK(){	
		
		double truePositive = 0;
		
		for (ClassificationResultTopK c : data) {
			
			if ( c.getObserved().contains(c.getExpected()) )
					
				truePositive += 1.0d;
		}
						
		return (truePositive) / (double) data.size();
		
	}



	
}
