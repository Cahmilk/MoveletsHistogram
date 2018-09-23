package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.Map;

public interface IQuality<T>{

	public void setData(Map<String,Double> data);
	
	public Map<String,Double> getData();
	
	public boolean hasZeroQuality();
	
	public int compareTo(T other);	
}
