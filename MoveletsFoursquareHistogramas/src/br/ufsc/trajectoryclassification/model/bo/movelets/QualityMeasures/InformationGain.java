package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import weka.attributeSelection.InfoGainAttributeEval;
import weka.classifiers.trees.j48.BinC45Split;
import weka.core.Attribute;
import weka.core.DenseInstance;
import weka.core.Instances;

public class InformationGain implements IQualityMeasure{

	private List<ITrajectory> trajectories;
	
	private List<String> labels;
	
	private Map<String, Long> classCounts;
	
	public InformationGain(List<ITrajectory> trajectories) {		
		this.trajectories = trajectories;	
		this.getInitialEntropy();
	}
	
	public void getInitialEntropy(){
		
		/*
		 * STEP 1: EXTRACT LABELS FROM TRAJECTORIES
		 */
		labels = new ArrayList<>();
		for (int j = 0; j < trajectories.size(); j++) {
			labels.add(trajectories.get(j).getLabel());
		}

		/*
		 * STEP 2: CALCULATE THE INITIAL ENTROPY FILLING THE INITIAL CLASS COUNT
		 */
		classCounts = labels.stream().collect(Collectors.groupingBy(e -> e, Collectors.counting()));		
	
	}
	
	@Override
	public void assesQuality(ISubtrajectory candidate){
		
		Map<String,Integer> classes = new HashMap<>();
		
		ArrayList<String> strClasses = new ArrayList<>(classCounts.keySet());
		for (int i = 0; i < strClasses.size(); i++) {
			classes.put(strClasses.get(i), i);
		}
		
		ArrayList<Attribute> atts = new ArrayList<Attribute>(2);
        
        atts.add(new Attribute("distance"));
        atts.add(new Attribute("class",strClasses));

        Instances dataRaw = new Instances("Dataset",atts,0);
        
        for (int i = 0; i < candidate.getDistances().length; i++) {
        	double[] instanceValue1 = new double[2];
            //instanceValue1[0] = candidate.getDistances()[i];
            instanceValue1[1] = classes.get(labels.get(i)).doubleValue();            
            dataRaw.add( new DenseInstance(1.0, instanceValue1) );
		}
                
        InfoGainAttributeEval evaluator = new InfoGainAttributeEval();
        
        BinC45Split binC45Split = new BinC45Split(0, 1, dataRaw.sumOfWeights(), true);
       
    	dataRaw.setClassIndex(1);
    	
    	double infogain = 0;
    	double splitpoint = 0;
    	double gap = 0;
    	
    	Instances dataNew = new Instances("Dataset",atts,0);
    	double[] leftSideInstance = new double[2];
    	leftSideInstance[0] = 0;
    	leftSideInstance[1] = classes.get(candidate.getTrajectory().getLabel()).doubleValue(); 
    	dataNew.add(new DenseInstance(1.0, leftSideInstance));
    	
    	double leftSideClassification = 0;
    	
    	try {    		
    		binC45Split.buildClassifier(dataRaw);    		
    		leftSideClassification = binC45Split.classifyInstance(dataNew.firstInstance());
    		
    		if (leftSideClassification == leftSideInstance[1] && binC45Split.infoGain() > 0.01 ){
    			infogain = binC45Split.infoGain();
    			splitpoint = binC45Split.splitPoint();
    			}
    		
    		if (splitpoint > 1) splitpoint = 1;
    		    								
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
   	
    	if (MyCounter.maxQuality < infogain){
    		MyCounter.maxQuality = infogain;
    		System.out.println(MyCounter.maxQuality);
    	}
    	
    	Map<String, Double> data = new HashMap<>();
    	data.put("quality", infogain);
    	data.put("splitpoint", splitpoint);
    	data.put("bestDistBetweenValues", 0.0);
    	data.put("size", 1.0 * candidate.getSize() );
    	data.put("start", 1.0 * candidate.getStart() );
    	data.put("tid", 1.0 * candidate.getTrajectory().getTid() );
    	
    	IQuality<InformationGainQuality> quality = new InformationGainQuality();
    	quality.setData(data);	    	
		candidate.setQuality(quality);
		    	
	}
	

}
