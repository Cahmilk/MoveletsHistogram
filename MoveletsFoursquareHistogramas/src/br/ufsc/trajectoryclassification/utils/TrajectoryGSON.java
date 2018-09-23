
package br.ufsc.trajectoryclassification.utils;

import java.util.HashMap;
import java.util.Map;

import br.ufsc.trajectoryclassification.model.vo.Trajectory;

public class TrajectoryGSON {
	
	public Map<String, Object> classOfT = new HashMap<>();
	
	public TrajectoryGSON(Trajectory t) {
		// TODO Auto-generated constructor stub
		classOfT.put("tid", t.getTid());
		classOfT.put("label", t.getLabel());	
	}

}
