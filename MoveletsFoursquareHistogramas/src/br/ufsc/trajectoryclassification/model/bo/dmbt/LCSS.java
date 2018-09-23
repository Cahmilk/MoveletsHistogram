package br.ufsc.trajectoryclassification.model.bo.dmbt;

import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class LCSS implements IDistanceMeasureBetweenTrajectories {

	private IDistanceMeasureBetweenPoints dmbp;
	
	private double[] thresholds;
	
	public LCSS(IDistanceMeasureBetweenPoints dmbp) {
		super();
		this.dmbp = dmbp;		
	}

	public int LongestCommonSubsequence(List<IPoint> t1, List<IPoint> t2, IDistanceMeasureBetweenPoints dmbp){
	        int[][] num = new int[t1.size()+1][t2.size()+1];  //2D array, initialized to 0

	        for (int i = 1; i <= t1.size(); i++)
	                for (int j = 1; j <= t2.size(); j++)
	                		if ( dmbp.haveMatch(t1.get(i-1), t2.get(j-1), thresholds) )	                        
	                                
	                				num[i][j] = 1 + num[i-1][j-1];
	                        else
	                                num[i][j] = Math.max(num[i-1][j], num[i][j-1]);

	        return num[t1.size()][t2.size()];
	}

	@Override
	public double getDistance(ITrajectory t1, ITrajectory t2) {
		
		int lcss = LongestCommonSubsequence(t1.getData(), t2.getData(), dmbp);
		int n = t1.getData().size();
		int m = t2.getData().size();
		
		//return 1.0 - (double) lcss / ((n + m) / 2.0);
		return 1.0 - (double) lcss / Math.min(n, m);

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
	public void setThresholds(double[] values) {
		// TODO Auto-generated method stub
		this.thresholds = values;
		
	}

	@Override
	public void setWeights(double[] values) {
		// TODO Auto-generated method stub
		
	}


}
