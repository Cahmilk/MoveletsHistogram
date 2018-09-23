package br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers;

import java.lang.reflect.InvocationTargetException;
import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.IPointFeature;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;

public class PointFeaturesExtraction {

	public static void fillTrajectoryPoints(ITrajectory trajectory, Description description) {

		List<String> pointFeaturesDesc = description.getPointFeaturesDesc();

		fillTrajectoryPoints(trajectory, pointFeaturesDesc);

	}

	public static void fillTrajectoryPoints(ITrajectory trajectory, List<String> strFeatures) {

		for (String strPointFeature : strFeatures) {
			try {
				/*
				 * Transform featureName to upperCase to match with the class
				 */
				String packageName = "br.ufsc.trajectoryclassification.model.bo.featureExtraction.";
				String featureClassName = strPointFeature.toUpperCase();

				IPointFeature pointFeature = (IPointFeature) Class.forName(packageName + featureClassName)
						.getConstructor().newInstance();

				pointFeature.fillPoints(trajectory);

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
			fillTrajectoryPoints(trajectory, description);
		}		
		
	}

	public static void fillAllTrajectories(List<ITrajectory> trajectories, List<String> strFeatures){

		for (ITrajectory trajectory : trajectories) {
			fillTrajectoryPoints(trajectory, strFeatures);
		}		
		
	}
}
