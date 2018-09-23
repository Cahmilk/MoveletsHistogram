package br.ufsc.trajectoryclassification.model.vo;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.bo.dmbt.IDistanceMeasureBetweenTrajectories;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;


public class Trajectory implements  ITrajectory{

	private int tid; 
		
	private List<IPoint> data;	
	
	private String label;
	
	private Map<String, IFeature> features;
	
	private Map<String,Double> attributes;
	
	public Trajectory(List<IPoint> data, String label) {
		super();
		this.data = data;
		this.label = label;
		this.features = new HashMap<>();
		this.attributes = new HashMap<>();
	}

	public Trajectory(int tid, List<IPoint> data, String label) {
		super();
		this.tid = tid;
		this.data = data;
		this.label = label;
		this.features = new HashMap<>();
		this.attributes = new HashMap<>();
	}

	
	public int getTid() {
		return tid;
	}

	public void setTid(int tid) {
		this.tid = tid;
	}

	public List<IPoint> getData() {
		return data;
	}

	public void setData(List<IPoint> data) {
		this.data = data;
	}

	public String getLabel() {
		return label;
	}

	public void setLabel(String label) {
		this.label = label;
	}

	@Override
	public String toString() {

		String string = new String();

		string += "Label: " + label + "\n";

		string += "Data: \n";

		for (int i = 0; i < data.size(); i++)
			string += data.get(i).toString() + "\n";

		return string;
	}
	
	public Map<String, IFeature> getFeatures() {
		return features;
	}
	
	@Override
	public Map<String, Double> getAttributes() {
		// TODO Auto-generated method stub
		return this.attributes;
	}
	
	@Override
	public IFeature getFeature(String featureName){
		return 	features.get(featureName);
	}


	public double compareToByData(ITrajectory otherTrajectory, IDistanceMeasureBetweenTrajectories dmbt) {
		return dmbt.getDistance(this, otherTrajectory);
	}

	public double compareToByLabel(Trajectory otherTrajectory) {

		if (this.getLabel().equals(otherTrajectory))
			return 0;
		else
			return 1;
	}
	
	public long getNumberOfPoints(){
		return data.size();		
	}
	
	public long getNumberOfValues(){
		
		return data.size() * data.get(0).getNumberOfValues();		
	}

	@Override
	public double compareTo(ITrajectory otherTrajectory, IDistanceMeasureBetweenTrajectories dmbt) {
		return dmbt.getDistance(this, otherTrajectory);
	}

}
