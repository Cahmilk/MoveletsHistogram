package br.ufsc.trajectoryclassification.model.bo.dmbt;

import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;


public class EDR implements IDistanceMeasureBetweenTrajectories {

	private IDistanceMeasureBetweenPoints dmbp;
	
	private double[] thresholds;

	public EDR(IDistanceMeasureBetweenPoints dmbp) {
		super();
		this.dmbp = dmbp;
	}

	
	private int getEditDistance(List<IPoint> t1, List<IPoint> t2, IDistanceMeasureBetweenPoints dmbp) {
		
	        int[][] num = new int[t1.size()+1][t2.size()+1];  //2D array, initialized to 0
	        
	        for (int i = 0; i < t1.size()+1; i++) {
	        	num[i][0] = i;
			}
	        
	        for (int j = 0; j < t2.size()+1; j++) {
	        	num[0][j] = j;
			}

	        for (int i = 1; i < t1.size()+1; i++)
	                for (int j = 1; j < t2.size()+1; j++){
	                	
	                	//double [] distances = dmbp.getDistance(t1.get(i-1), t2.get(j-1));
	                	boolean haveMatch = dmbp.haveMatch(t1.get(i-1), t2.get(j-1), thresholds);
	                	
	                	//if (haveMatch(distances, thresholds))
	                	if (haveMatch)
                			num[i][j] = num[i-1][j-1];
                        else {
                            num[i][j] = Math.min(num[i-1][j], num[i][j-1]);
                            num[i][j] = 1 + Math.min(num[i][j], num[i-1][j-1]);
                        	}
	                }
	        return num[t1.size()][t2.size()];
	}
	
	private boolean haveMatch(double [] distances, double [] thresholds){
		
		for (int i = 0; i < distances.length; i++) {
			
			if (distances[i] > thresholds[i])
			
				return false;
			
		}
		
		return true;
		
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
		int edr = getEditDistance(t1.getData(), t2.getData(), dmbp);
		int n = t1.getData().size();
		int m = t2.getData().size();
		
		//return 2.0 * edr / (n+m);
		return  (double) edr / Math.max(n, m);
	}


	public void setThresholds(double[] values) {
		this.thresholds = values;
	}

	@Override
	public void setWeights(double[] values) {
		// TODO Auto-generated method stub
		
	}
	
}
