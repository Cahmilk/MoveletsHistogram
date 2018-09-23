package br.ufsc.trajectoryclassification.model.bo.dmbs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbp.DMBP;
import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.description.SubtrajectoryComparisonDesc;

public class DMSeuclidean implements IDistanceMeasureForSubtrajectory {

	private IDistanceMeasureBetweenPoints dmbp;

	private SubtrajectoryComparisonDesc subtrajectoryComparisonDesc;

	private FeatureComparisonDesc featureComparisonDescToPoints;

	private List<FeatureComparisonDesc> featureComparisonDescToSubtrajectories;

	private Description description;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();

	public DMSeuclidean(Description description) {
		super();
		this.description = description;
		this.subtrajectoryComparisonDesc = description.getSubtrajectoryComparisonDesc();
		this.dmbp = DMBP.getDMBPfromDescription(description);

		List<FeatureComparisonDesc> list = subtrajectoryComparisonDesc.getFeatureComparisonDesc();

		featureComparisonDescToSubtrajectories = new ArrayList<>();

		for (FeatureComparisonDesc featureComparisonDesc : list) {
			if (featureComparisonDesc.getText().equalsIgnoreCase("points"))
				featureComparisonDescToPoints = featureComparisonDesc;
			else
				featureComparisonDescToSubtrajectories.add(featureComparisonDesc);
		}

	}
		
	@Override
	public Pair<ISubtrajectory,double[]> getBestAlignment(ISubtrajectory s, ITrajectory t) {
		
		double[] maxValues = new double[s.getPointFeatures().length];
		Arrays.fill(maxValues, Double.MAX_VALUE);
		
		/*
		 * If subtrajectory is largest than trajectory return maximum distance
		 */
		if (s.getSize() > t.getData().size())
			return new Pair<>(null,maxValues);

		/*
		 * Compute min distance
		 */
		if (featureComparisonDescToPoints != null) {
			if (featureComparisonDescToSubtrajectories.size() > 0)
				return getBestAlignmentByPointAndSubtrajectoryFeatures(s, t);
			else
				return getBestAlignmentByPointFeatures(s, t);
		} else {
			if (featureComparisonDescToSubtrajectories.size() > 0)
				return getBestAlignmentSubtrajectoryFeatures(s, t);
			else
				return new Pair<>(null,maxValues);
		}
		

	}
	
	private int getBestAlignmentByRanking(double[][] ranksForT, int[] comb) {
		
		/* Sum of ranks */
		double[] rankMerged = new double[ranksForT[0].length];
		for (int i = 0; i < comb.length; i++) {
			for (int j = 0; j < ranksForT[0].length; j++) {
				rankMerged[j] += ranksForT[i][j];
			}
		}
		
		// Get the first occurence of min value
		int minRankIndex = 0;
		for (int j = 1; j < rankMerged.length; j++) {
			if (rankMerged[j] < rankMerged[minRankIndex])
				minRankIndex = j;
		}
		/*			
		int minIdx = IntStream.range(0,rankMerged.length)
	            .reduce((i,j) -> rankMerged[i] > rankMerged[j] ? j : i)
	            .getAsInt();
	            */ 
		/*
		System.out.println(Arrays.toString(rankMerged));
		System.out.println(minIdx);
		*/
		return minRankIndex;
	}

	
	private boolean firstVectorGreaterThanTheSecond(double [] first, double [] second){
		
		for (int i = 0; i < first.length; i++) {
			if (first[i] <= second[i])
				return false;
		}
		
		return true;
	} 
	
	public double[] getDistance(ISubtrajectory s1, ISubtrajectory s2) {
		
		if (s1.getSize() != s2.getSize()){
			System.out.println("The two subtrajectories have different length.");
			return null;
		}
		
		if ( !Arrays.equals( s1.getPointFeatures(), s2.getPointFeatures() ) ){
			System.out.print("The two subtrajectories have different point features.");
			return null;
		}
		
		int[] comb = s1.getPointFeatures();
		double currentSum[] = new double[comb.length];
		
		Arrays.fill(currentSum, 0);
		
		double[] values = new double[dmbp.getNumberOfFeatures()];		
		for (int i = 0; i < s1.getSize(); i++) {
			
			values = dmbp.getDistance(s1.getData().get(i), s2.getData().get(i));

			for (int k = 0; k < comb.length; k++) {					
				if (currentSum[k] != Double.MAX_VALUE && values[k] != Double.MAX_VALUE)
					currentSum[k] += values[comb[k]] * values[comb[k]];
				else {
					currentSum[k] = Double.MAX_VALUE;
				}
			}
		}
		
		for (int k = 0; k < comb.length; k++) {
			
			double distance = currentSum[k];
			
			currentSum[k] = 
					(distance != Double.MAX_VALUE) ? Math.sqrt(distance / s1.getSize()) 
												   : Double.MAX_VALUE;
			
		} // for (int j = 0; j < comb.length; j++)


		return currentSum;		

	}
	
	private Pair<ISubtrajectory,double[]> getBestAlignmentByPointFeatures(ISubtrajectory s, ITrajectory t) {

		double[] maxValues = new double[dmbp.getNumberOfFeatures()];
		Arrays.fill(maxValues, Double.MAX_VALUE);
				
		if (s.getData().size() > t.getData().size())
			return new Pair<>(null,maxValues);

		List<IPoint> menor = s.getData();
		List<IPoint> maior = t.getData();
		
		int diffLength = maior.size() - menor.size();		
				
		int[] comb = s.getPointFeatures();
		double currentSum[] = new double[comb.length];
		double[] values = new double[dmbp.getNumberOfFeatures()];
		double[][] distancesForT = new double[comb.length][diffLength+1];
						
		double[] x = new double[comb.length];
		Arrays.fill(x, Double.MAX_VALUE);
				
		for (int i = 0; i <= diffLength; i++) {

			Arrays.fill(currentSum, 0);
						
			for (int j = 0; j < menor.size(); j++) {

				values = dmbp.getDistance(menor.get(j), maior.get(i + j));

				for (int k = 0; k < comb.length; k++) {					
					if (currentSum[k] != Double.MAX_VALUE && values[k] != Double.MAX_VALUE)
						currentSum[k] += values[comb[k]] * values[comb[k]];
					else {
						currentSum[k] = Double.MAX_VALUE;
					}
				}
				
				
				if (firstVectorGreaterThanTheSecond(currentSum, x) ){
					for (int k = 0; k < comb.length; k++) {
						currentSum[k] = Double.MAX_VALUE;
					}					
					break;					
				} 											
				
			}
			
			if (firstVectorGreaterThanTheSecond(x, currentSum) ){
				for (int k = 0; k < comb.length; k++) {
					x[k] = currentSum[k];					
					}				
			}
			
			for (int k = 0; k < comb.length; k++) {
				distancesForT[k][i] = currentSum[k];
			}
		}
		
		double[][] ranksForT = new double[distancesForT.length][];
		
		for (int k = 0; k < comb.length; k++) {
			ranksForT[k] = rankingAlgorithm.rank(distancesForT[k]);
		} // for (int k = 0; k < numberOfFeatures; k++)
		
		
		int bestPosition = getBestAlignmentByRanking(ranksForT,comb);
		
		double[] bestAlignment = new double[comb.length];
		
		for (int j = 0; j < comb.length; j++) {
			
			double distance = distancesForT[j][bestPosition];
			
			bestAlignment[j] = 
					(distance != Double.MAX_VALUE) ? Math.sqrt(distance / menor.size()) 
												   : Double.MAX_VALUE;
			
		} // for (int j = 0; j < comb.length; j++)
		
		int start = bestPosition;
		int end = bestPosition + menor.size() - 1;
		
		return new Pair<>(new Subtrajectory(start, end , t), bestAlignment);
	}
	
	
	private Pair<ISubtrajectory,double[]> getBestAlignmentByPointFeaturesBackup(ISubtrajectory s, ITrajectory t) {

		double[] maxValues = new double[dmbp.getNumberOfFeatures()];
		Arrays.fill(maxValues, Double.MAX_VALUE);
				
		if (s.getData().size() > t.getData().size())
			return new Pair<>(null,maxValues);

		List<IPoint> menor = s.getData();
		List<IPoint> maior = t.getData();
		
		int diffLength = maior.size() - menor.size();		
				
		int[] comb = s.getPointFeatures();
		double currentSum[] = new double[comb.length];
		double[] values = new double[dmbp.getNumberOfFeatures()];
		double[][] distancesForT = new double[comb.length][diffLength+1];
		
		for (int i = 0; i <= diffLength; i++) {

			Arrays.fill(currentSum, 0);

			for (int j = 0; j < menor.size(); j++) {

				values = dmbp.getDistance(menor.get(j), maior.get(i + j));

				for (int k = 0; k < comb.length; k++) {					
					if (currentSum[k] != Double.MAX_VALUE && values[k] != Double.MAX_VALUE)
						currentSum[k] += values[comb[k]] * values[comb[k]];
					else {
						currentSum[k] = Double.MAX_VALUE;
					}
				}

			}

			for (int k = 0; k < comb.length; k++) {
				distancesForT[k][i] = currentSum[k];
			}
		}
		
		double[][] ranksForT = new double[distancesForT.length][];
		
		for (int k = 0; k < comb.length; k++) {
			ranksForT[k] = rankingAlgorithm.rank(distancesForT[k]);
		} // for (int k = 0; k < numberOfFeatures; k++)
		
		
		int bestPosition = getBestAlignmentByRanking(ranksForT,comb);
		
		double[] bestAlignment = new double[comb.length];
		
		for (int j = 0; j < comb.length; j++) {
			
			double distance = distancesForT[j][bestPosition];
			
			bestAlignment[j] = 
					(distance != Double.MAX_VALUE) ? Math.sqrt(distance / menor.size()) 
												   : Double.MAX_VALUE;
			
		} // for (int j = 0; j < comb.length; j++)
		
		int start = bestPosition;
		int end = bestPosition + menor.size() - 1;
		
		return new Pair<>(new Subtrajectory(start, end , t), bestAlignment);
	}
	private Pair<ISubtrajectory,double[]> getBestAlignmentSubtrajectoryFeatures(ISubtrajectory s, ITrajectory t) {
		
		/* NOT IMPLEMENTED YET
		 * */
		
		return null;
	}

	private Pair<ISubtrajectory,double[]> getBestAlignmentByPointAndSubtrajectoryFeatures(ISubtrajectory s, ITrajectory t) {
		
		/* NOT IMPLEMENTED YET
		 * */
		
		return null;
	}
	

	@Override
	public IDistanceMeasureBetweenPoints getDMBP() {
		// TODO Auto-generated method stub
		return dmbp;
	}

	public Description getDescription() {
		return description;
	}
}
