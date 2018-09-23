package br.ufsc.trajectoryclassification.model.bo.dmbs;

import java.lang.reflect.InvocationTargetException;

import br.ufsc.trajectoryclassification.model.vo.description.Description;

public class DMS{
	
	
	public static IDistanceMeasureForSubtrajectory getDMSfromDescription(Description description){
		
		/* Transform distanceMeasure name to lowerCase to match with the class 
		 * */
		String packageName = "br.ufsc.trajectoryclassification.model.bo.dmbs.";
		String dmsClassName =  "DMS" + description.getSubtrajectoryComparisonDesc().getSubtrajectoryDistace().toLowerCase();
		
		try {
		
			IDistanceMeasureForSubtrajectory dms = 
					(IDistanceMeasureForSubtrajectory) Class.forName(packageName + dmsClassName).getConstructor(Description.class).newInstance(description);
		
			return dms;
			
		} catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
				| NoSuchMethodException | SecurityException | ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return null;
	}	
	
}
	
