package br.ufsc.trajectoryclassification.utils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQuality;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureQuality;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public class SubtrajectoryGSON {
	
	private int start;

	private int end;
	
	private String name;
 
	private int trajectory;
	
	private String label;

	private HashMap<String, IFeature> features;
		
	private HashMap<String, Double> maxValues;
	
	private int[] pointFeatures;
	
	private double[] splitpoints;
	
	private double[][] distances;
	
	private int[] positions;
	
	private Map<String,Double> quality;

	private static String ConstructName(ISubtrajectory movelet) {
			
			return "sh_TID" + movelet.getTrajectory().getTid() + "_START" + movelet.getStart() + "_SIZE" + movelet.getSize() + "_CLASS" + movelet.getTrajectory().getLabel();
			
	}
	
	public SubtrajectoryGSON(int start, int end, int trajectory, String label,
			HashMap<String, IFeature> features, int[] pointFeatures, double [] splitpoints, double[][] distances, ISubtrajectory[] bestAlignments, IQuality quality, Description description, String name) {
		super();
		this.start = start;
		this.end = end;
		this.trajectory = trajectory;
		this.label = label;
		this.features = features;		
		this.distances = distances;
		this.positions = Arrays.asList(bestAlignments).stream().mapToInt(e -> (e!=null) ? e.getStart() : -1).toArray();
		this.pointFeatures = pointFeatures;
		this.splitpoints = splitpoints;
		this.quality = quality.getData();
		this.maxValues = new HashMap<>();
		this.name = name;
		
		for (FeatureComparisonDesc featureComparisonDesc : description.getPointComparisonDesc().getFeatureComparisonDesc()) {
			maxValues.put(featureComparisonDesc.getText(), featureComparisonDesc.getMaxValue());				
		}

		for (FeatureComparisonDesc featureComparisonDesc : description.getSubtrajectoryComparisonDesc().getFeatureComparisonDesc()) {
			maxValues.put(featureComparisonDesc.getText(), featureComparisonDesc.getMaxValue());				
		}

	}

	public int getStart() {
		return start;
	}

	public void setStart(int start) {
		this.start = start;
	}

	public int getEnd() {
		return end;
	}

	public void setEnd(int end) {
		this.end = end;
	}

	public int getTrajectory() {
		return trajectory;
	}

	public void setTrajectory(int trajectory) {
		this.trajectory = trajectory;
	}

	public String getLabel() {
		return label;
	}

	public void setLabel(String label) {
		this.label = label;
	}

	public double[] getSplitpoints() {
		return splitpoints;
	}
	
	public void setSplitpoints(double[] splitpoints) {
		this.splitpoints = splitpoints;
	}

	public HashMap<String, IFeature> getFeatures() {
		return features;
	}

	public void setFeatures(HashMap<String, IFeature> features) {
		this.features = features;
	}

	public double[][] getDistances() {
		return distances;
	}

	public void setDistances(double[][] distances) {
		this.distances = distances;
	}

	public Map<String, Double> getQuality() {
		return quality;
	}
	
	public void setQuality(Map<String, Double> quality) {
		this.quality = quality;
	}
	
		
	public int[] getPointFeatures() {
		return pointFeatures;
	}

	public void setPointFeatures(int[] pointFeatures) {
		this.pointFeatures = pointFeatures;
	}

	public static SubtrajectoryGSON fromSubtrajectory(ISubtrajectory s, Description description){
		
		String name = ConstructName(s);
		
		return new SubtrajectoryGSON(s.getStart(), s.getEnd(), s.getTrajectory().getTid(), 
				s.getTrajectory().getLabel(), s.getFeatures(), s.getPointFeatures(), s.getSplitpoints(), s.getDistances(), s.getBestAlignments(),
				s.getQuality(), description, name);
		
	}
	
	public ISubtrajectory toSubtrajectory(List<ITrajectory> trajectories){
		
		ITrajectory t = trajectories.stream().filter(e -> e.getTid() == this.trajectory).collect(Collectors.toList()).get(0);
		
		ISubtrajectory s = new Subtrajectory(start, end, t, pointFeatures, distances[0].length);
		
		LeftSidePureQuality lspq = new LeftSidePureQuality();
		lspq.setData(quality);		
		s.setDistances(distances);		
		s.setQuality(lspq);
		s.setSplitpoints(splitpoints);		
		
		return s;
		
	}
	

}
