
package br.ufsc.trajectoryclassification.model.vo.description;

import java.util.HashMap;
import java.util.Map;

public class FeatureComparisonDesc {

    private String distance;
    private Double maxValue = new Double(-1);
    private String text;
    private Map<String, Object> additionalProperties = new HashMap<String, Object>();

    public String getDistance() {
		return distance;
	}

	public void setDistance(String distance) {
		this.distance = distance;
	}

	public Double getMaxValue() {
        return maxValue;
    }

    public void setMaxValue(Double maxValue) {
        this.maxValue = maxValue;
    }

    public String getText() {
        return text;
    }

    public void setText(String text) {
        this.text = text;
    }

    public Map<String, Object> getAdditionalProperties() {
        return this.additionalProperties;
    }

    public void setAdditionalProperty(String name, Object value) {
        this.additionalProperties.put(name, value);
    }

}
