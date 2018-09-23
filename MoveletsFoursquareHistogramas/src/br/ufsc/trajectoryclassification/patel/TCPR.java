package br.ufsc.trajectoryclassification.patel;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class TCPR {

	private List<ITrajectory> train;
	private List<ITrajectory> test;
	
	private List<Vertex> V;
	
	private List<Edge> E;
	
	
	public TCPR(List<ITrajectory> train, List<ITrajectory> test) {
		super();
		this.train = train;
		this.test = test;
	}
	
	public TCPR(List<ITrajectory> train) {
		super();
		this.train = train;
	}
	
	public void buildInitialGraph(){
		
		V = new ArrayList<>();
		E = new ArrayList<>();
		
		for (ITrajectory t : train) {	
			
			List<IPoint> points = t.getData();
			List<Vertex> v = new ArrayList<>();
			
			for (IPoint p: points){
				Vertex vi = new Vertex();
				vi.addPoint(p, t);
				v.add(vi);
			}
			
			for (int i = 0; i < v.size()-1; i++) {	
				Edge e = new Edge(v.get(i), v.get(i+1), t.getLabel());
				e.addPoints(
						v.get(i).getPoints().get(0), 
						v.get(i+1).getPoints().get(0),
						v.get(i).getTrajectories().get(0));				
				E.add(e);				
				v.get(i).getOut().add(e);
				v.get(i+1).getIn().add(e);
			}
			
			V.addAll(v);
		}		
		
	}
	
	public Double MDLcost(){
		
		double LM = V.size() + E.size();
		
		double HV = V.stream().mapToDouble(e -> e.getEntropy()).sum();
		
		double HE = E.stream().mapToDouble(e -> e.getEntropy()).sum();
		
		return 0.0;
		
	}
		
	public Pair<Pair<Vertex,Vertex>,Double> MDLgainV(){
		
		for (Vertex vi : V) {
			
			for (Vertex vj : V) {
				
				
				
			}
			
		}
		
		
		return null;
	}
	
	public void mergeVertices(){
		/*
		while (MDLgainV() >= 0){
			
		}		
			*/	
	}
	
	
	public void mergeEdges(){
		
	}
	
	
	public void run (){
		
		buildInitialGraph();
		
		System.out.println(V.size());

		System.out.println(E.size());
		
		mergeVertices();
	}
	

}
