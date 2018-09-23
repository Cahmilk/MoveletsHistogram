package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.util.List;

public class ClassificationResultTopK {
	
	private List<String> observed;
	private String expected;

	public ClassificationResultTopK(List<String> observed, String expected) {
		super();
		this.observed = observed;
		this.expected = expected;
	}

	public List<String> getObserved() {
		return observed;
	}

	public String getExpected() {
		return expected;
	}

}