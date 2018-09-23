
package br.ufsc.trajectoryclassification.model.vo.description;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Description {

    private List<ReadsDesc> readsDesc = null;
    private List<String> pointFeaturesDesc = null;
    private List<String> subtrajectoryFeaturesDesc = null;
    private List<String> trajectoryFeaturesDesc = null;
    private PointComparisonDesc pointComparisonDesc;
    private SubtrajectoryComparisonDesc subtrajectoryComparisonDesc;
    private Map<String, Object> additionalProperties = new HashMap<String, Object>();

    public List<ReadsDesc> getReadsDesc() {
        return readsDesc;
    }

    public void setReadsDesc(List<ReadsDesc> readsDesc) {
        this.readsDesc = readsDesc;
    }

    public List<String> getPointFeaturesDesc() {
        return pointFeaturesDesc;
    }

    public void setPointFeaturesDesc(List<String> pointFeaturesDesc) {
        this.pointFeaturesDesc = pointFeaturesDesc;
    }

    public List<String> getSubtrajectoryFeaturesDesc() {
        return subtrajectoryFeaturesDesc;
    }

    public void setSubtrajectoryFeaturesDesc(List<String> subtrajectoryFeaturesDesc) {
        this.subtrajectoryFeaturesDesc = subtrajectoryFeaturesDesc;
    }

    public List<String> getTrajectoryFeaturesDesc() {
        return trajectoryFeaturesDesc;
    }

    public void setTrajectoryFeaturesDesc(List<String> trajectoryFeaturesDesc) {
        this.trajectoryFeaturesDesc = trajectoryFeaturesDesc;
    }

    public PointComparisonDesc getPointComparisonDesc() {
        return pointComparisonDesc;
    }

    public void setPointComparisonDesc(PointComparisonDesc pointComparisonDesc) {
        this.pointComparisonDesc = pointComparisonDesc;
    }

    public SubtrajectoryComparisonDesc getSubtrajectoryComparisonDesc() {
        return subtrajectoryComparisonDesc;
    }

    public void setSubtrajectoryComparisonDesc(SubtrajectoryComparisonDesc subtrajectoryComparisonDesc) {
        this.subtrajectoryComparisonDesc = subtrajectoryComparisonDesc;
    }

    public Map<String, Object> getAdditionalProperties() {
        return this.additionalProperties;
    }

    public void setAdditionalProperty(String name, Object value) {
        this.additionalProperties.put(name, value);
    }
    
    

}
