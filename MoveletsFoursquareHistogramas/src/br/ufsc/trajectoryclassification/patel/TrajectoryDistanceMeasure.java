package br.ufsc.trajectoryclassification.patel;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.TimeAsInteger;

public class TrajectoryDistanceMeasure {
	
	public boolean equalSamplingTimes(TimeAsInteger time1, TimeAsInteger time2){
		return time1.getValue() == time2.getValue();		
	}
	
	public boolean contains(List<TimeAsInteger> list, TimeAsInteger time){
		
		for (TimeAsInteger timei : list) {
			if (equalSamplingTimes(timei, time)) return true;
			if (timei.getValue() > time.getValue()) return false;
		}
		
		return false;
	}
	
	public List<TimeAsInteger> getSamplingTimes(ITrajectory t1, ITrajectory t2){
		
		List<TimeAsInteger> list = new ArrayList<>();
		
		for (IPoint p : t1.getData()) {			
			TimeAsInteger time = (TimeAsInteger) p.getFeature("time");
			list.add(time);
		}

		for (IPoint p : t2.getData()) {			
			TimeAsInteger time = (TimeAsInteger) p.getFeature("time");
			if (!list.contains(time));
				list.add(time);
		}
		
		Collections.sort(list, new Comparator<TimeAsInteger>() {
            @Override
            public int compare(TimeAsInteger time1, TimeAsInteger time2) {
                return time2.getValue() - time1.getValue();
            }
        });

		return list;
	}
	
	
	public double getDistance(ITrajectory t1, ITrajectory t2){
	
		/* 1. Get the set of times 
		 * */
		List<TimeAsInteger> samplingTimes = getSamplingTimes(t1, t2);
		
		/* 2. For each time measure the distance between points.
		 * If one trajectory hasn't these time point, perform a interpolation 
		 * */
		List<IPoint> pointsT1 = t1.getData();
		List<IPoint> pointsT2 = t2.getData();
		int iT1 = 0;
		int iT2 = 0;
		
		for (TimeAsInteger time : samplingTimes) {
			
			if ( !time.equals(pointsT1.get(iT1).getFeature("time") ) )
					/* Interpolar t1 */;
			
			if ( !time.equals(pointsT2.get(iT2).getFeature("time") ) )
					/* Interpolar t2 */;
			
			
		}
		
		/* 3. Return the weighted sum of distances, where the weigth
		 * is the sampling time of the imediate neigbors points.
		 * */
		
		return 0;
	}
	
}
