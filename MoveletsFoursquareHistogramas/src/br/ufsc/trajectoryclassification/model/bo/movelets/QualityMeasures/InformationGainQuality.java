package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.HashMap;
import java.util.Map;

public class InformationGainQuality implements IQuality<InformationGainQuality>{

	private Map<String, Double> data;
	
	public InformationGainQuality() {
		data = new HashMap<>();
	}
	
	public Map<String, Double> getData() {
		return data;
	}
	
	@Override
	public void setData(Map<String, Double> data) {
		// TODO Auto-generated method stub
		this.data = data;
	}
	
	@Override
	public int compareTo(InformationGainQuality other) {
				
		// Primeiro criterio		
		int infogainComparison = Double.compare(this.getData().get("quality"), other.getData().get("quality"));
		if (infogainComparison != 0)
			// Ordem decrescente
			return (-1) * infogainComparison;
		
		int diffSplitpoint = Double.compare(this.getData().get("splitpoint"), other.getData().get("splitpoint"));
		if (diffSplitpoint != 0)	
		// Ordem crescente
		return diffSplitpoint;
				
		//else, se for igual
		// Terceiro criterio
		int diffSize = Double.compare(this.getData().get("size"), other.getData().get("size"));
		if (diffSize != 0)	
		// Ordem crescente
		return diffSize;
		
		
		int diffStart = Double.compare(this.getData().get("start"), other.getData().get("start"));
		if (diffStart != 0)	
		// Ordem crescente
		return diffStart;
		
		int diffTid = Double.compare(this.getData().get("tid"), other.getData().get("tid"));
		if (diffTid != 0)	
		// Ordem crescente
		return diffTid;
		
		// else, se for igual
		// Segundo criterio			
		//int distBetweenValuesComparison = Double.compare(this.getData().get("bestDistBetweenValues"), other.getData().get("bestDistBetweenValues"));				
		//if (distBetweenValuesComparison != 0)	
			// Ordem decrescente
		//	return (-1) * distBetweenValuesComparison;
		
		
//		
//		int diffTid = Integer.compare(o1.getTrajectory().getTid(), o2.getTrajectory().getTid());
//		if (diffTid != 0)	
//			// Ordem crescente
//			return diffTid < 0 ? -1 : 1;
		
		return 0;
	}

	@Override
	public boolean hasZeroQuality() {		
		
		return (data.get("quality") == 0);
		
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return data.toString();
	}

}
