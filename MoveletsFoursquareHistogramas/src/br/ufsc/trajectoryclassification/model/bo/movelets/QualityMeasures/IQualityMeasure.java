package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;

public interface IQualityMeasure {
	
	public void assesQuality(ISubtrajectory candidate);

}
