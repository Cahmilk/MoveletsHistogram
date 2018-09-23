package br.ufsc.trajectoryclassification.model.bo.featureExtraction;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.description.PointComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.description.SubtrajectoryComparisonDesc;

public class ComparisonFeatureParameterEstimation {
	
	private List<ITrajectory> trajectories;
	
	private Description description;
	
	private double factor = 1;

		
	public ComparisonFeatureParameterEstimation(List<ITrajectory> trajectories, Description description) {
		super();
		this.trajectories = trajectories;
		this.description = description;
	}
	
	public void setFactor(double factor) {
		this.factor = factor;
	}

	public void estimateParmeters(){
		
		/* After this method the object Description will be modified
		 * */
		pointComparisonFeatureParameterEstimation(description.getPointComparisonDesc());
		
		subtrajectoryComparisonFeatureParameterEstimation(description.getSubtrajectoryComparisonDesc());
	}

	private void subtrajectoryComparisonFeatureParameterEstimation(SubtrajectoryComparisonDesc subtrajectoryComparisonDesc) {
		// TODO Auto-generated method stub
		
		List<FeatureComparisonDesc> list = subtrajectoryComparisonDesc.getFeatureComparisonDesc();
		
		for (FeatureComparisonDesc featureComparisonDesc : list) {
			
			if (featureComparisonDesc.getMaxValue() == -2){
				double maxValue = 0;
				
				switch (featureComparisonDesc.getText().toUpperCase()) {			
				
				case "STARTSPACE":
					maxValue = getMaxValueByMean("space");
					break;
				case "ENDSPACE":
					maxValue = getMaxValueByMean("space");
					break;
				case "ACCELERATION":
					maxValue = getMaxValueByMean("acceleration");
					break;
				case "AVERAGESPEED":
					maxValue = getMaxValueByMean("speed");
					break;
			
				default:
					System.out.println("Estimation for subtrajectory feature "+featureComparisonDesc.getText().toUpperCase()+" do not supported yet.");
					break;
				} 
				
				System.out.println("Max value estimated for feature: "+featureComparisonDesc.getText().toUpperCase() + "; value: "+maxValue);
			
				featureComparisonDesc.setMaxValue(maxValue);
			}
		}
		
	}

	private void pointComparisonFeatureParameterEstimation(PointComparisonDesc pointComparisonDesc) {
		
		List<FeatureComparisonDesc> list = pointComparisonDesc.getFeatureComparisonDesc();
		
		for (FeatureComparisonDesc featureComparisonDesc : list) {
			
			if (featureComparisonDesc.getMaxValue() == -2){
				double maxValue = 0;
				
				switch (featureComparisonDesc.getText().toUpperCase()) {				
				case "SPACE":
					maxValue = getMaxValueByMean("space");
					break;
				case "ACCELERATION":
					maxValue = getMaxValueByMean("acceleration") / 2;
					break;
				case "SPEED":
					maxValue = getMaxValueByMean("speed") / 2;
					break;
				case "STAYTIME":
					maxValue = getMaxValueByMean("time") / 2;
					break;		
				case "TURNANGLE":
					maxValue = getMaxValueByMean("turnangle") / 2;
					break;
				case "SINUOSITY":
					maxValue = getMaxValueByMean("sinuosity") / 2;
					break;	
				case "TRAVELEDDISTANCE":
					maxValue = getMaxValueByMean("space");
					break;	
				default:
					System.out.println("Estimation for point feature "+featureComparisonDesc.getText().toUpperCase()+" do not supported yet.");
					break;
					
				} 
				
				System.out.println("Max value estimated for feature: "+featureComparisonDesc.getText().toUpperCase() + "; value: "+maxValue);
				
				featureComparisonDesc.setMaxValue(maxValue);
			} 
		}
		// TODO Auto-generated method stub
		
	}

	private double getMaxValueByMean(String strFeature){
		
		double sum = 0;
		long n = 0;
		
		for (int i = 0; i < trajectories.size(); i++) {		
			
			ITrajectory t = trajectories.get(i);
			
			for (int j = 1; j < t.getData().size(); j++) {
				double d = t.getData().get(j-1).getFeature(strFeature).getDistanceTo(
						t.getData().get(j).getFeature(strFeature), null);
				if (!Double.isNaN(d))
					sum += d;
				else 
					n--;
				
			}
			
			n += t.getData().size() ;
			
		}
				
		return (sum / n) * factor; 
	}

	private double getMaxValueByMedian(String strFeature){

		List<Double> list = new ArrayList<>();
		
		for (int i = 0; i < trajectories.size(); i++) {		
			
			ITrajectory t = trajectories.get(i);
			
			for (int j = 1; j < t.getData().size(); j++) {
				
				list.add(
						t.getData().get(j-1).getFeature(strFeature).getDistanceTo(
								t.getData().get(j).getFeature(strFeature), null) );
				
			}
			
		}
				
		Collections.sort(list);
		return list.get(list.size()/2) * factor; 
	}
}
