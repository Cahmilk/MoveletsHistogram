package br.ufsc.trajectoryclassification.patel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class Edge {
	
	private Vertex start;
	
	private Vertex end;
	
	private List<IPoint> startPoints = new ArrayList<>();;
	
	private List<ITrajectory> trajectories = new ArrayList<>();

	private List<IPoint> endPoints = new ArrayList<>();
	
	private Map<String,Integer> countLabels = new HashMap<>();
	
	private Double time;
	
	private String label;

	public Edge(Vertex start, Vertex end, String label) {
		super();
		this.start = start;
		this.end = end;		
		this.label = label;
		this.fillTime();
	}
	
	public void addPoints(IPoint start, IPoint end, ITrajectory t){
		this.startPoints.add(start);
		this.endPoints.add(end);
		this.trajectories.add(t);
		this.incCountLabel(t.getLabel());
	}	
	
	public void incCountLabel(String key){
		
		if (countLabels.containsKey(key)){
			Integer value = countLabels.get(key);
			countLabels.put(key, value++);
		} else {
			countLabels.put(key, 1);
		}	
			
	}
	
		
	public void fillTime(){
		
		if (start == null || end == null) return;
				
	}

	public Vertex getStart() {
		return start;
	}

	public void setStart(Vertex start) {
		this.start = start;
	}

	public Vertex getEnd() {
		return end;
	}

	public void setEnd(Vertex end) {
		this.end = end;
	}

	public Double getTime() {
		return time;
	}

	public void setTime(Double time) {
		this.time = time;
	}

	public String getLabel() {
		return label;
	}

	public void setLabel(String label) {
		this.label = label;
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
