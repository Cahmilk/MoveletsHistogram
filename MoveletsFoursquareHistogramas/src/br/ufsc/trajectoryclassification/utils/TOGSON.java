package br.ufsc.trajectoryclassification.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class TOGSON {
	
	private List<Map<String,Object>> classes;
	
	private List <SubtrajectoryGSON> shapelets;
	
	public TOGSON() {
		// TODO Auto-generated constructor stub
	}
	
	

	public TOGSON(List<Map<String, Object>> classes, List<SubtrajectoryGSON> shapelets) {
		super();
		this.classes = classes;
		this.shapelets = shapelets;
	}



	public void setClasses(List<Map<String, Object>> classes) {
		this.classes = classes;
	}
	
	public void setShapelets(List<SubtrajectoryGSON> shapelets) {
		this.shapelets = shapelets;
	}
	
	public List<SubtrajectoryGSON> getShapelets() {
		return shapelets;
	}
	
	
	public List<Map<String, Object>> getClasses() {
		return classes;
	}
}
