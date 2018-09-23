package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.util.List;

public class ClassificationResultsEval {
	
	private List<ClassificationResult> data;
	
	private int numTruePositives = 0;
	
	private int numTrueNegatives = 0;
	
	private int numFalsePositives = 0;
	
	private int numFalseNegatives = 0;
	
	private String positiveClass = "1";

	public ClassificationResultsEval(List<ClassificationResult> data) {
		super();
		this.data = data;
		performConfusion();
	}

	public ClassificationResultsEval(List<ClassificationResult> data, String positiveClass) {
		super();
		this.data = data;
		this.positiveClass = positiveClass;
		performConfusion();
	}
	
	public void performConfusion(){
		
		for (ClassificationResult c : data) {
			
			if (c.getExpected().equals(c.getObserved()))
					
					if (c.getExpected().equals(positiveClass))
						
						numTruePositives++;
					else
						numTrueNegatives++;
			else 
					if (c.getExpected().equals(positiveClass))			
						numFalsePositives++;
					else
						numFalseNegatives++;
		}
				
	}

	public List<ClassificationResult> getData() {
		return data;
	}
	
	public double errorRate(){	
		
		double errorRate = 0;
		
		for (ClassificationResult c : data) {
			
			if (!c.getExpected().equals(c.getObserved()))
					
				errorRate += 1.0d;
		}
						
		return (errorRate) / (double) data.size();
		
	}
	
	public double accuracy(){
		
		double truePositive = 0;
		
		for (ClassificationResult c : data) {
			
			if (c.getExpected().equals(c.getObserved()))
					
				truePositive += 1.0d;
		}
						
		return (truePositive) / (double) data.size();
		
	}
	
	
	
	public double precision(){
		return (numTruePositives) / (double) (numTruePositives + numFalsePositives);
	}

	public double recall(){
		return (numTruePositives) / (double) (numTruePositives + numFalseNegatives);
	}

	public int numTruePositives() {
		return numTruePositives;
	}

	public int numTrueNegatives() {
		return numTrueNegatives;
	}

	public int numFalsePositives() {
		return numFalsePositives;
	}

	public int numFalseNegatives() {
		return numFalseNegatives;
	}

	public String getPositiveClass() {
		return positiveClass;
	}
	
	@Override
	public String toString() {
		String str = new String();
		if (data != null){
			
			str += errorRate() +
					", " + precision() + 
					", " + recall() +
					", " + numTruePositives() + 
					", " + numTrueNegatives() + 
					", " + numFalsePositives() + 
					", " + numFalseNegatives();
			
		}
		return str;		
	}
	
	
	
}
