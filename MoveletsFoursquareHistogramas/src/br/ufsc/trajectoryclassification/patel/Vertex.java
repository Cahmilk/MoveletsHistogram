package br.ufsc.trajectoryclassification.patel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class Vertex {
		
	private List<IPoint> points = new ArrayList<>();;
	
	private List<ITrajectory> trajectories = new ArrayList<>();
	
	private Map<String,Integer> countLabels = new HashMap<>();
	
	private List<Edge> in = new ArrayList<>();
	
	private List<Edge> out = new ArrayList<>();
	

	public Vertex() {
		// TODO Auto-generated constructor stub
	}
	
	public void addPoint(IPoint point, ITrajectory trajectory){
		this.points.add(point);
		this.trajectories.add(trajectory);		
		this.incCountLabel(trajectory.getLabel());
	}
	
	public List<IPoint> getPoints() {
		return points;
	}
	
	public List<ITrajectory> getTrajectories() {
		return trajectories;
	}
	
	public List<Edge> getIn() {
		return in;
	}
	
	public List<Edge> getOut() {
		return out;
	}
	
	public void incCountLabel(String key){
		
		if (countLabels.containsKey(key)){
			Integer value = countLabels.get(key);
			countLabels.put(key, value++);
		} else {
			countLabels.put(key, 1);
		}	
			
	}
	
	
	public Double getEntropy(){
		
		double sum = 0.0;
		
		double n = trajectories.size(); 
		
		for (String key : countLabels.keySet()) {
			double prop = countLabels.get(key) / n;
			sum += prop * Math.log(prop) / Math.log(2);
		}
		
		return (-1) * sum;
	}

}
