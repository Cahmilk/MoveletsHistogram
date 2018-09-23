package br.ufsc.trajectoryclassification.model.bo.dmbt;


import java.util.Arrays;
import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;


public class MDDTW implements IDistanceMeasureBetweenTrajectories {

	private IDistanceMeasureBetweenPoints dmbp;
	
	private double[] thresholds;
	
	private double[] weights;


	public MDDTW(IDistanceMeasureBetweenPoints dmbp) {
		super();
		this.dmbp = dmbp;
	}

	public double getDTW(List<IPoint> t1, List<IPoint> t2, IDistanceMeasureBetweenPoints dmbp) {
		
        double[][] num = new double[t1.size()+1][t2.size()+1];  //2D array, initialized to 0
        
        for (int i = 1; i < t1.size()+1; i++) {
        	num[i][0] = Double.MAX_VALUE;
		}
        
        for (int j = 1; j < t2.size()+1; j++) {
        	num[0][j] = Double.MAX_VALUE;
		}
        
        num[0][0] = 0;
        
        for (int i = 1; i <= t1.size(); i++)
        	
                for (int j = 1; j <= t2.size(); j++){
                	
                		double[] costs = dmbp.getDistance(t1.get(i-1), t2.get(j-1));
                		double cost = combineCosts(costs,thresholds, weights);
                				
        		        num[i][j] = Math.min(num[i-1][j], num[i][j-1]);
                        num[i][j] = cost + Math.min(num[i][j], num[i-1][j-1]);
                        
                        }
                
        return num[t1.size()][t2.size()];
	}
	
	private double combineCosts(double[] distances, double[] thresholds, double[] weights){
		
		double normalized = 0.0;
		
		double weighted = 0.0;
		
		double costsSum = 0.0;
		
		double sumWeights = Arrays.stream(weights).sum();

		for (int i = 0; i < distances.length; i++) {
				
				normalized = (thresholds[i] > 0) ? (distances[i] / thresholds[i]) : distances[i];
				weighted = normalized * (weights[i] / sumWeights);
				costsSum += weighted;

		}
		
		return costsSum;
		
	}

	@Override
	public double getDistance(ITrajectory t1, ITrajectory t2) {
		
		return getDTW(t1.getData(), t2.getData(), dmbp);

	}

	@Override
	public IDistanceMeasureBetweenPoints getDistanceMeasureBetweenPoints() {
		// TODO Auto-generated method stub
		return dmbp;
	}

	@Override
	public void setDistanceMeasureBetweenPoints(IDistanceMeasureBetweenPoints dmbp) {
		// TODO Auto-generated method stub
		this.dmbp = dmbp;
	}

	public Object clone() throws CloneNotSupportedException{
		return this;
	}

	public void setThresholds(double[] values) {
		this.thresholds = values;
	}

	public void setWeights(double[] values) {
		// TODO Auto-generated method stub
		this.weights = values;
	}
	
	
}
