package br.ufsc.trajectoryclassification.model.dao;

import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;

public interface ITrajectoryDAO {
		
	public List<ITrajectory> loadFromDir(String dirpath, Description description);
	
	public ITrajectory loadFromFile(String filename);
	
	public ITrajectory loadFromFile(String filename, Description description);
		
	public Description loadDescription(String filepath);

}
