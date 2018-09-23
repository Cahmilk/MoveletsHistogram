package br.ufsc.trajectoryclassification.model.bo.similarity;

public class ClassificationResult {
	
	private String observed;
	private String expected;

	public ClassificationResult(String observed, String expected) {
		super();
		this.observed = observed;
		this.expected = expected;
	}

	public String getObserved() {
		return observed;
	}

	public String getExpected() {
		return expected;
	}

}