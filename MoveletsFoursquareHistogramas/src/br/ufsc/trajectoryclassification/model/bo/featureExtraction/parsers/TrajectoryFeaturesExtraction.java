package br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers;

import java.lang.reflect.InvocationTargetException;
import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ITrajectoryFeature;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;

public class TrajectoryFeaturesExtraction {
	
	public TrajectoryFeaturesExtraction() {
		// TODO Auto-generated constructor stub
	}
	
	public static void fillTrajectory(ITrajectory trajectory, Description description){
		
		List<String> trajectoryFeaturesDesc = description.getTrajectoryFeaturesDesc();
		
		for (String strTrajectoryFeature : trajectoryFeaturesDesc) {
			try {
				/* Transform featureName to upperCase to match with the class 
				 * */
				String packageName = "br.ufsc.trajectoryclassification.model.bo.featureExtraction.";
				String featureClassName =  strTrajectoryFeature.toUpperCase();
				
				ITrajectoryFeature trajectoryFeature = 
						(ITrajectoryFeature) Class.forName(packageName + featureClassName).getConstructor().newInstance();
				
				trajectoryFeature.fillTrajectory(trajectory);
				
				
			} catch (InstantiationException | IllegalAccessException | IllegalArgumentException
					| InvocationTargetException | NoSuchMethodException | SecurityException
					| ClassNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
	}
	
	public static void fillAllTrajectories(List<ITrajectory> trajectories, Description description){

		for (ITrajectory trajectory : trajectories) {
			fillTrajectory(trajectory, description);
		}		
		
	}

}
