package br.ufsc.trajectoryclassification.model.vo;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.ml.clustering.Clusterable;

import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQuality;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public interface ISubtrajectory extends Clusterable {
	
	public int getStart();
	
	public int getEnd();

	public int getSize();
	
	public List<IPoint> getData();
	
	public ITrajectory getTrajectory();
	
	public HashMap<String, IFeature> getFeatures();
	
	public void setSplitpointData(Map<String, double[]> splitpointData);
	
	public Map<String, double[]> getSplitpointData();
	
	public Map<Integer, double[]> getMdist();

	public IFeature getFeature(String featureName);
	
	public void setQuality(IQuality quality);
	
	public IQuality getQuality();
	
	public boolean[] getGoodTrajectories();
	
	public double[] getSplitpoints();
	
	public void setSplitpoints(double[] splitpoints);
	
	public int[] getPointFeatures();
	
	public void setPointFeatures(int[] pointFeatures);

	public double[] getMaxDistances();
	
	public void setMaxDistances(double[] maxDistances);
	
	public ISubtrajectory[] getBestAlignments(); 

	public void setBestAlignments(ISubtrajectory[] bestAligments); 
	
	public double[][] getDistances();
	
	public void setDistances(double[][] distances);

	public void createMDist();
	

}
