package br.ufsc.trajectoryclassification.model.vo;

import java.time.LocalTime;
import java.util.HashMap;
import java.util.List;

import br.ufsc.trajectoryclassification.model.bo.dmbp.IDistanceMeasureBetweenPoints;
import br.ufsc.trajectoryclassification.model.vo.description.ReadsDesc;
import br.ufsc.trajectoryclassification.model.vo.features.FoursquareVenue;
import br.ufsc.trajectoryclassification.model.vo.features.GowallaCheckin;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.model.vo.features.Nominal;
import br.ufsc.trajectoryclassification.model.vo.features.Numeric;
import br.ufsc.trajectoryclassification.model.vo.features.Space2D;
import br.ufsc.trajectoryclassification.model.vo.features.TimeAsLocalTime;

public class Point implements IPoint {
		
	private HashMap<String,IFeature> features;

	public Point() {
		// TODO Auto-generated constructor stub
		this.features = new HashMap<>();
	}
	
	
	public HashMap<String, IFeature> getFeatures() {
		return features;
	}
	
	@Override
	public IFeature getFeature(String featureName){
		return 	features.get(featureName);
	}
	
	@Override
	public String toString() {				
		return features.toString();
	}

	public double[] compareTo(IPoint otherPoint, IDistanceMeasureBetweenPoints dmbp) {
		return dmbp.getDistance(this, otherPoint);		
	}

	@Override
	public int getNumberOfValues() {
		// TODO Auto-generated method stub
		return features.size();
	}

	public static Point loadFromTextAndDesc(String line, List<ReadsDesc> readsDesc) {

		Point point = new Point();
		
		String[] rows = line.split(",");
		
		for (int i = 0; i < rows.length; i++) {
			
			String str = rows[i];
			String key = readsDesc.get(i).getText();
			IFeature<?> value = null;
			
			if (readsDesc.get(i).getType().equalsIgnoreCase("localtime"))
				value = new TimeAsLocalTime( LocalTime.parse(str) );
			else if (readsDesc.get(i).getType().equalsIgnoreCase("Space2D"))
				value = new Space2D(str);
			else if (readsDesc.get(i).getType().equalsIgnoreCase("nominal"))
				value = new Nominal(str);
			else if (readsDesc.get(i).getType().equalsIgnoreCase("numeric"))
				value = new Numeric(Double.parseDouble(str));
			else if (readsDesc.get(i).getType().equalsIgnoreCase("foursquarevenue"))
				value = new FoursquareVenue(str);		
			else if (readsDesc.get(i).getType().equalsIgnoreCase("GowallaCheckin"))
				value = new GowallaCheckin(str);
			point.getFeatures().put(key, value);			
		}

		return point;
	}
	
}
