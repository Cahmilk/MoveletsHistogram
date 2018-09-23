package br.ufsc.trajectoryclassification.patel;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.dao.ITrajectoryDAO;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.utils.UnzipUtility;

public class Patel {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static int nthreads = 1;
	private static int minSize = 1;
	private static int maxSize = -1;	
	private static String strQualityMeasure = "IG";
 	private static double factor = 1;
 	private static double dimReduction = 0.0;
	private static boolean multiattribute = false;
	private static ITrajectoryDAO trajectoryDAO = new TrajectoryDAO();

	
	public static void configure(String[] args) {

		for (int i = 0; i < args.length; i = i + 2) {
			String key = args[i];
			String value = args[i + 1];
			switch (key) {
			case "-curpath":
				CURRENT_DIR = value;
				break;
			case "-respath":
				RESULT_DIR = value;
				break;
			case "-descfile":
				DESCRIPTION_FILE = value;
				break;
			case "-nt":
				nthreads = Integer.valueOf(value);
				break;
			case "-dr":
				dimReduction = Double.valueOf(value);
				break;

			default:
				System.err.println("Parâmetro " + key + " inválido.");
				System.exit(1);
				return;
			}
		}

	}

	public static String showConfiguration() {

		String str = new String();

		str += "Starting running patel algorithm " + System.getProperty("line.separator");

		str += "Configurations:" + System.getProperty("line.separator");

		str += "\tBase directory:	    " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:    " + RESULT_DIR + System.getProperty("line.separator");
		
		str += "\tDescription file :    " + DESCRIPTION_FILE + System.getProperty("line.separator");

		str += "\tAllowed Threads:      " + nthreads + System.getProperty("line.separator");		

		return str;

	}
	
	public static List<ITrajectory> loadTrajectories(String dirPath, Description description){
		
		UnzipUtility.unzip(dirPath + ".zip", dirPath);
		
		List<ITrajectory> trajectories = trajectoryDAO.loadFromDir(dirPath,description);
		
		try {
			FileUtils.deleteDirectory(new File(dirPath));			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return trajectories;	
		
	}

	public static void main(String[] args) {

		if (args.length == 0) return;
		/*
		 * STEP 1. Configura parâmetros de entrada
		 */
		configure(args);
		System.out.println(showConfiguration());

		String representation = "r7";
		
		/*
		 * STEP 2. Configura os endereços onde estão os arquivos de dados e o
		 * endereço de resultados
		 */
		String trainDirPath = CURRENT_DIR + "data-" + representation + "/train";
		String testDirPath = CURRENT_DIR + "data-" + representation + "/test";
		
		String DESCRIPTION_FILE_NAME = FilenameUtils.removeExtension(
				new File(DESCRIPTION_FILE).getName());
		
		String resultDirPath = RESULT_DIR + "data-" + representation + "/results/Patel/";
		
		
		System.out.println("\nStarting...");
				
		/*
		 * STEP 4. Carrega as trajetórias em memória
		 */
		String descriptionPathFile = DESCRIPTION_FILE;
		if (descriptionPathFile == null)
			descriptionPathFile = CURRENT_DIR + "data-" + representation + "/description.json";
		
		/* Load description file and train and test trajectories */
		Description description = trajectoryDAO.loadDescription(descriptionPathFile);
		List<ITrajectory> train = loadTrajectories(trainDirPath, description);
		List<ITrajectory> test = loadTrajectories(testDirPath, description);
		
		/* Extract Local Feature */
		WeightPointFeature weightPointFeature = new WeightPointFeature();

		weightPointFeature.fillTrajectories(train);	
		
		TrajectoryDistanceMeasure trajectoryDistanceMeasure = new TrajectoryDistanceMeasure();
		double distance = trajectoryDistanceMeasure.getDistance(train.get(0), train.get(1));
		
		System.out.println(distance);
		
		TCPR tcpr = new TCPR(train,test);
		
		tcpr.run();

//		Utils.writeTrajectoriesToGSON(train, description, resultDirPath + "all.json");
		
//		Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
				
	}

}
