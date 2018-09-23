package br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers;

import java.lang.reflect.InvocationTargetException;
import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ISubtrajectoryFeature;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;

public class SubtrajectoryFeaturesExtraction {
	
	public SubtrajectoryFeaturesExtraction() {
		// TODO Auto-generated constructor stub
	}
	
	public static void fillSubtrajectory(ISubtrajectory subtrajectory, Description description){
		
		List<String> subtrajectoryFeaturesDesc = description.getSubtrajectoryFeaturesDesc();
		
		for (String strTrajectoryFeature : subtrajectoryFeaturesDesc) {
			try {
				/* Transform featureName to upperCase to match with the class 
				 * */
				String packageName = "br.ufsc.trajectoryclassification.model.bo.featureExtraction.";
				String featureClassName =  strTrajectoryFeature.toUpperCase();
				
				ISubtrajectoryFeature subtrajectoryFeature = 
						(ISubtrajectoryFeature) Class.forName(packageName + featureClassName).getConstructor().newInstance();
				
				subtrajectoryFeature.fillSubtrajectory(subtrajectory);				
				
			} catch (InstantiationException | IllegalAccessException | IllegalArgumentException
					| InvocationTargetException | NoSuchMethodException | SecurityException
					| ClassNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}		
	}
	
	public static void fillAllSubtrajectories(List<ISubtrajectory> subtrajectories, Description description){

		for (ISubtrajectory subtrajectory : subtrajectories) {
			fillSubtrajectory(subtrajectory, description);
		}		
		
	}

}
