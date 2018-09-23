
package br.ufsc.trajectoryclassification.model.vo.description;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SubtrajectoryComparisonDesc {

    private String subtrajectoryDistance;
    private List<FeatureComparisonDesc> featureComparisonDesc = null;
    private Map<String, Object> additionalProperties = new HashMap<String, Object>();

    public String getSubtrajectoryDistace() {
        return subtrajectoryDistance;
    }

    public void setSubtrajectoryDistace(String subtrajectoryDistace) {
        this.subtrajectoryDistance = subtrajectoryDistace;
    }
    
    public List<FeatureComparisonDesc> getFeatureComparisonDesc() {
		return featureComparisonDesc;
	}

	public void setFeatureComparisonDesc(List<FeatureComparisonDesc> featureComparisonDesc) {
		this.featureComparisonDesc = featureComparisonDesc;
	}

	public Map<String, Object> getAdditionalProperties() {
        return this.additionalProperties;
    }

    public void setAdditionalProperty(String name, Object value) {
        this.additionalProperties.put(name, value);
    }

}
