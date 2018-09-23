package br.ufsc.trajectoryclassification.model.bo.histogram;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.Pair;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.utils.Utils;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;

public class Histogram {

	private IDistanceMeasureForSubtrajectory dmbt;
	private List<ISubtrajectory> movelets;
	private List<ITrajectory> train;
	private List<ITrajectory> test;
	private String filepath_train;
	private String filepath_test;
	private ArrayList<ArrayList<ArrayList<Integer>>> movPositions;
	
	public Histogram(List<ITrajectory> trajectories_test, List<ITrajectory> trajectories_train, List<ISubtrajectory> movelets, IDistanceMeasureForSubtrajectory dmbt, String filepath_test, String filepath_train) {
		// TODO Auto-generated constructor stub
		
		this.dmbt = dmbt;
		this.movelets = movelets;
		this.test = trajectories_test;	
		this.train = trajectories_train;		
		this.filepath_train = filepath_train;
		this.filepath_test = filepath_test;
		
	}
	
	//Construction of movelet name
	private String ConstructName(int counter, String line, ISubtrajectory movelet) {
		
		if(counter!=0)
			line = line+",";
		
		line = line + "sh_TID" + movelet.getTrajectory().getTid() + "_START" + movelet.getStart() + "_SIZE" + movelet.getSize() + "_CLASS" + movelet.getTrajectory().getLabel();
		
		return line;
	}
	
	//Get each movelet name
	private String getMoveletsName(List<ISubtrajectory> movelets) {
		
		int counter = 0;
		String line = "";
		
		for(ISubtrajectory movelet:movelets) {
			
			line = ConstructName(counter, line, movelet);
			counter++;
		}
		
		return line;
		
	}
	
	private String getHeader(List<ISubtrajectory> movelets){
		
		String line = "";
		line = getMoveletsName(movelets);
		
		line = line+",label"+System.lineSeparator();
		return line;
	}
	
	
	private List<Integer> getMovPosInTraj(ISubtrajectory movelet, ITrajectory trajectory, double[][] distances_mov) {
		
		//Selects the number of point features, and splitpoint values calculated inside the movelets
		int[] point_features = movelet.getPointFeatures();
		double[] splitpoints = movelet.getSplitpoints();
		
		List<Integer> positions = new ArrayList<>();
		
		//para todas as
		for(int k = 0; k<distances_mov[0].length; k++) {
			
			int attribute_counter = 0;
			
			for(int j = 0; j < distances_mov.length; j++)
				if(distances_mov[j][k]<=splitpoints[j])
					attribute_counter++;
			
			//If all the attributes are under the splitpoint, then it counts the occurrence.
			if(attribute_counter==point_features.length)
				positions.add(k);	
				
		}
		
		return positions;
	}
	
	private Map<List<List<Integer>>,String> getFrequences(ITrajectory trajectory) {
		
		int i = 0;
		String line = "";
		
		List<List<Integer>> mov_pos_relation = new ArrayList<List<Integer>>();
		
		//For each movelet, verify if occurrence on the actual trajectory
		for(ISubtrajectory movelet:this.movelets) {	
			
			double[][] distances_mov = Utils.getDistancesFromMoveletsForHistogram(movelet, trajectory, dmbt);
			
			if(i!=0)
				line = line+",";

			//mov_counter will count the movelet occurrence
			List<Integer> MovPositions = getMovPosInTraj(movelet, trajectory, distances_mov);
			
			line = line + MovPositions.size();
			mov_pos_relation.add(MovPositions);
			
			i++;
			
		}
		
		line = line+ "," + trajectory.getLabel() + System.lineSeparator();
		

		Map<List<List<Integer>>,String> example = new HashMap<List<List<Integer>>,String>();
		
		example.put(mov_pos_relation, line);
		
		return example;
	}
	
	private void writeHistogramInFile(String filepath, List<ITrajectory> trajectories) {
		
		BufferedWriter writer;
		
		try {
			
			File file = new File(filepath);
			file.getParentFile().mkdirs();
			writer = new BufferedWriter(new FileWriter(file));
			Pair<List<List<Integer>>,String> pair = new Pair<List<List<Integer>>, String>();
					
			Map<List<List<Integer>>,String> ref = new HashMap<List<List<Integer>>,String>();
			
			writer.write(getHeader(movelets));
			
			for(ITrajectory trajectory:trajectories) {
				ref = getFrequences(trajectory);
				writer.write(ref.get(key));
				
			}
				
			
			//Close the writer
			writer.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	//Fazer aqui ele chamar tanto pra treino quanto pra teste.
	public void getHistograms() {
		
		writeHistogramInFile(this.filepath_test, this.test);
		writeHistogramInFile(this.filepath_train, this.train);
		
	}
	

}
