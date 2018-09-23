package br.ufsc.trajectoryclassification.model.bo.dmbp;

import java.lang.reflect.InvocationTargetException;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ITrajectoryFeature;
import br.ufsc.trajectoryclassification.model.vo.description.Description;

public class DMBP {

	public static IDistanceMeasureBetweenPoints getDMBPfromDescription(Description description){
				
		/* Transform distanceMeasure name to lowerCase to match with the class 
		 * */
		String packageName = "br.ufsc.trajectoryclassification.model.bo.dmbp.";
		String dmbpClassName =  "DMBP" + description.getPointComparisonDesc().getPointDistance().toLowerCase();
		
		try {
		
			IDistanceMeasureBetweenPoints dmbp = 
					(IDistanceMeasureBetweenPoints) Class.forName(packageName + dmbpClassName).getConstructor(Description.class).newInstance(description);
		
			return dmbp;
		} catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
				| NoSuchMethodException | SecurityException | ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return null;
	}	
	
}
