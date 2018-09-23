package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.HashMap;
import java.util.Map;

public class LeftSidePureQuality implements IQuality<LeftSidePureQuality>{

	private Map<String, Double> data;
	
	public LeftSidePureQuality() {
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
	public int compareTo(LeftSidePureQuality other) {
				
		// First it compares the quality score		
		int quality = Double.compare(this.getData().get("quality"), other.getData().get("quality"));
		if (quality != 0)
			// in descending order
			return (-1) * quality;
		
		// It continues if both split distance are the same
		// Fourth it compares the movelet size
		int diffSize = Double.compare(this.getData().get("size"), other.getData().get("size"));
		if (diffSize != 0)
			// in ascending order
			return  diffSize;

		// It continues if both sizes are the same
		// Fourth it compares the movelet size
		int diffDimensions = Double.compare(this.getData().get("dimensions"), other.getData().get("dimensions"));
		if (diffDimensions != 0)
			// in ascending order
			return  diffDimensions;
				
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
