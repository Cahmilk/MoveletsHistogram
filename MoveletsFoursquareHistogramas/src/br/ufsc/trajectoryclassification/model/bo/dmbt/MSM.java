package br.ufsc.trajectoryclassification.model.bo.dmbt;

import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import weka.core.Utils;


public class MSM implements IDistanceMeasureBetweenTrajectories {

	private IDistanceMeasureBetweenPoints dmbp;
	
	private double[] thresholds;
	
	private double[] weights;

	public MSM(IDistanceMeasureBetweenPoints dmbp) {
		super();
		this.dmbp = dmbp;
	}

	
	private double[][] getDistanceMatrix(List<IPoint> t1, List<IPoint> t2, IDistanceMeasureBetweenPoints dmbp) {
		
	        double[][] num = new double[t1.size()][t2.size()];  //2D array, initialized to 0        

	        for (int i = 0; i < t1.size(); i++)
	        	
	                for (int j = 0; j < t2.size(); j++){
	                	
	                	double [] distances = dmbp.getDistance(t1.get(i), t2.get(j));
	                	
	                	num[i][j] = partialMatch(distances, thresholds, weights);
                        
	                }
	        
	        return num;
	}
	

	private double[] maxByRow(RealMatrix rm){
		
		double[] maxByRow = new double[rm.getRowDimension()];
		
		for (int i = 0; i < rm.getRowDimension(); i++) {
			maxByRow[i] = getMax(rm.getRow(i));
		}
		
		return maxByRow;
	}
	
	private double[] maxByColumn(RealMatrix rm){
		
		double[] maxByColumn = new double[rm.getColumnDimension()];
		
		for (int i = 0; i < rm.getColumnDimension(); i++) {
			maxByColumn[i] = getMax(rm.getColumn(i));
		}
		
		return maxByColumn;
	}	
	
	private double getMax(double[] values){
		
		double max = values[0];
		
		for (int i = 1; i < values.length; i++) {
			if (values[i] > max)
				max = values[i];
		}
		
		return max;
		
	}
	
	
	private double matches(List<IPoint> t1, List<IPoint> t2, IDistanceMeasureBetweenPoints dmbp){
		
		double[][] matrix = getDistanceMatrix(t1, t2, dmbp);
		
		RealMatrix rm = new Array2DRowRealMatrix(matrix);
		
		double[] maxByRow = maxByRow(rm); 
		
		double[] maxByColumn = maxByColumn(rm);
		
		double sumRow = Utils.sum(maxByRow);
		double sumColumn = Utils.sum(maxByColumn);

		return (sumRow + sumColumn);
		
	}
	
	
	
	private double partialMatch(double [] distances, double [] thresholds, double[] weights){
		
		double partialMatch = 0;
		
		double sumWeights = Arrays.stream(weights).sum();
		
		for (int i = 0; i < distances.length; i++) {
			
			if (distances[i] <= thresholds[i])
			
				partialMatch += (weights[i] / sumWeights);
			
		}
		
		return partialMatch;
		
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

	@Override
	public double getDistance(ITrajectory t1, ITrajectory t2) {
		double matches = matches(t1.getData(), t2.getData(), dmbp);
		
		int n = t1.getData().size();
		int m = t2.getData().size();
		
		return  1.0 - (matches / (n + m));
	}


	public void setThresholds(double[] values) {
		this.thresholds = values;
	}

	@Override
	public void setWeights(double[] values) {
		// TODO Auto-generated method stub
		this.weights = values;
	}
	
}
