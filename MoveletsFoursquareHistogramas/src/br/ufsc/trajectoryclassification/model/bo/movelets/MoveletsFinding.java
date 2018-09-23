package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.List;
import java.util.concurrent.Callable;

import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;


public class MoveletsFinding implements Callable<Integer> {

	private List<ISubtrajectory> candidates;

	private List<ITrajectory> trajectories;

	private IDistanceMeasureForSubtrajectory dmbt;
			
	public MoveletsFinding(List<ISubtrajectory> candidates, List<ITrajectory> trajectories,
			IDistanceMeasureForSubtrajectory dmbt) {
		super();
		this.candidates = candidates;
		this.trajectories = trajectories;
		this.dmbt = dmbt;		
	}

	@Override
	public Integer call() throws Exception {

		run();

		return 0;

	}

	public void run() {

		for (ISubtrajectory candidate : candidates) {

			if (candidate.getDistances() == null)		
				ComputeDistances(candidate);
		
		}
	
	}

	public void ComputeDistances(ISubtrajectory candidate) {
		
		/* This pairs will store the subtrajectory of the best alignment 
		 * of the candidate into each trajectory and the distance 
		 * */
		Pair<ISubtrajectory, double[]> bestAlignment;
		
		double[][] trajectoryDistancesToCandidate = new double[candidate.getPointFeatures().length][trajectories.size()];
		
		ISubtrajectory[] bestAlignments = new ISubtrajectory[trajectories.size()];
				
		/* It calculates the distance of trajectories to the candidate
		 */
		for (int i = 0; i < trajectories.size(); i++) {
			
			bestAlignment = dmbt.getBestAlignment(candidate, trajectories.get(i));
						
			bestAlignments[i] = bestAlignment.getFirst();
			
			for (int j = 0; j < candidate.getPointFeatures().length; j++) {
				trajectoryDistancesToCandidate[j][i] = bestAlignment.getSecond()[j];
			}
						
		}
		
		candidate.setDistances(trajectoryDistancesToCandidate);
		candidate.setBestAlignments(bestAlignments);
	}


}
