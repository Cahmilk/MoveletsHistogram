package br.ufsc.trajectoryclassification.model.dao;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Point;
import br.ufsc.trajectoryclassification.model.vo.Trajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.ReadsDesc;


public class TrajectoryDAO implements ITrajectoryDAO {
	

	public ITrajectory loadFromFile(String filename, Description description) {

		String line;

		List<IPoint> data = new ArrayList<>();

		int tid = -1;
		String label = new String();
		
		BufferedReader bufferedReader;

		try {
			File file = new File(filename);
			// Extracting information from file name
			String[] filenameSplitted = file.getName().split("[ |\\.]");
			tid = Integer.valueOf( filenameSplitted[1].substring(1) );
			label = filenameSplitted[2].substring(1);
			
			bufferedReader = new BufferedReader(new FileReader(filename));
			
			while ((line = bufferedReader.readLine()) != null) {
				data.add(readRow(line,description.getReadsDesc()));
			}

			bufferedReader.close();

		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return new Trajectory(tid, data, label);
	}

	public IPoint readRow(String line, List<ReadsDesc> readsDesc) {

		return Point.loadFromTextAndDesc(line, readsDesc);

	}
	

	public List<ITrajectory> loadFromDir(String dirpath, Description description) {

		File folder = new File(dirpath);
		List<ITrajectory> list = new ArrayList<>();

		if (folder.isDirectory()) {
			//System.out.print("Reading Directory...");
			File[] listFiles = folder.listFiles();
			Arrays.sort(listFiles, new Comparator<File>() {
	            @Override
	            public int compare(File o1, File o2) {	                
	                return o1.getName().compareTo(o2.getName());
	            }
			});
						
			for (File file : listFiles) {

				String[] splittedName = file.getName().split("\\.");

				if (splittedName.length == 2 && splittedName[1].compareTo("r2") == 0) {

					list.add(loadFromFile(file.getAbsolutePath(),description));

				}
			}
			
			//System.out.println("Terminated. Loaded " + list.size() + " files.");
			//System.out.println("Number of Points: " + list.stream().mapToLong(n -> n.getNumberOfPoints()).sum() );
			//System.out.println("Number of Values: " + list.stream().mapToLong(n -> n.getNumberOfValues()).sum() );

		} else
			System.err.println("Dirpath are not a directory.");
		
		return list;
	}
	
	

	@Override
	public Description loadDescription(String filepath) {
				
		Reader reader;
		Description description = null;
		
		try {
			
			reader = new InputStreamReader(
					new FileInputStream(filepath), "UTF-8");
			
			Gson gson = new GsonBuilder().create();
	        
	        description = gson.fromJson(reader, Description.class);	        
	        
		} catch (UnsupportedEncodingException | FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
               
		return description;
	}

	@Override
	public ITrajectory loadFromFile(String filename) {
		// TODO Auto-generated method stub
		return null;
	}
	

}
