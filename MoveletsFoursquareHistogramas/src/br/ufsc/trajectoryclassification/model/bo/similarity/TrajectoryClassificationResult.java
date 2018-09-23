package br.ufsc.trajectoryclassification.model.bo.similarity;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;

import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class TrajectoryClassificationResult {

	private ITrajectory trajectory;
	
	private List<Pair<ITrajectory,Double>> topk;

	public ITrajectory getTrajectory() {
		return trajectory;
	}

	public void setTrajectory(ITrajectory trajectory) {
		this.trajectory = trajectory;
	}
	
	public List<Pair<ITrajectory, Double>> getTopk() {
		return topk;
	}
	
	public void setTopk(List<Pair<ITrajectory, Double>> topk) {
		this.topk = topk;
	}
	
	public TrajectoryClassificationResult(ITrajectory trajectory, List<Pair<ITrajectory, Double>> topk) {
		super();
		this.trajectory = trajectory;
		this.topk = topk;
	}
	
	public boolean isInTop1(){
		
		return trajectory.getLabel().equals(topk.get(0).getFirst().getLabel());
		
	}
	
	public boolean isInTopK(){
		
		for (int i = 0; i < topk.size(); i++) {
			
			if (trajectory.getLabel().equals(topk.get(i).getFirst().getLabel()))
				
				return true;		
		}
		
		return false;
		
	}
	
	public boolean isInTopKClass(int kclass){
		
		HashSet<String> topKClasses = new HashSet<>();
						
		for (int i = 0; i < topk.size(); i++) {
			
			topKClasses.add( topk.get(i).getFirst().getLabel() );
						
			if (topKClasses.size() == kclass)
				
				break;
			
		}
				
		return topKClasses.contains(trajectory.getLabel());
		
	}
	
}
