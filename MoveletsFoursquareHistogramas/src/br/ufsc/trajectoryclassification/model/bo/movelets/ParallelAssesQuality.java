package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.List;
import java.util.concurrent.Callable;

import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;

public class ParallelAssesQuality implements Callable<Integer> {

	private IQualityMeasure qualityMeasure;
	
	private List<ISubtrajectory> candidates;
	
	
	
	public ParallelAssesQuality(IQualityMeasure qualityMeasure, List<ISubtrajectory> candidates) {
		super();
		this.qualityMeasure = qualityMeasure;
		this.candidates = candidates;
	}

	@Override
	public Integer call() throws Exception {
		run();
		return 0;		
	}

	private void run() {
		// TODO Auto-generated method stub
		for (ISubtrajectory candidate : candidates) {
			qualityMeasure.assesQuality(candidate);			
		}
	}
	
	
	

}
