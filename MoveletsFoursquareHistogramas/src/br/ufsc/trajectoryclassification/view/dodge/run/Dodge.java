package br.ufsc.trajectoryclassification.view.dodge.run;

import java.io.File;
import java.util.List;

import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.dodge.DodgeFeatures;
import br.ufsc.trajectoryclassification.model.dao.ITrajectoryDAO;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.utils.Utils;

public class Dodge {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static int nthreads = 1;
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

			default:
				System.err.println("Parâmetro " + key + " inválido.");
				System.exit(1);
				return;
			}
		}

	}

	public static String showConfiguration() {

		String str = new String();

		str += "Starting running Dodge feature extraction " + System.getProperty("line.separator");

		str += "Configurations:" + System.getProperty("line.separator");

		str += "\tBase directory:	    " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:    " + RESULT_DIR + System.getProperty("line.separator");
		
		str += "\tDescription file :    " + DESCRIPTION_FILE + System.getProperty("line.separator");

		str += "\tAllowed Threads:      " + nthreads + System.getProperty("line.separator");		

		return str;

	}
	
	

	public static void main(String[] args) {

		if (args.length == 0) return;
		/*
		 * STEP 1. Configura parâmetros de entrada
		 */
		configure(args);
		System.out.println(showConfiguration());
		
		if (DESCRIPTION_FILE == null) return;
		
		String DESCRIPTION_FILE_NAME = FilenameUtils.removeExtension(
				new File(DESCRIPTION_FILE).getName());
		
		String resultDirPath = RESULT_DIR + "/Dodge/";
		
		/*
		 * STEP 2. Configura os endereços onde estão os arquivos de dados e o
		 * endereço de resultados
		 */
		String trainDirPath = CURRENT_DIR + "/train";
		String testDirPath = CURRENT_DIR + "/test";
				
		
		String descriptionPathFile = DESCRIPTION_FILE;
				
		System.out.println("\nStarting...");
				
		/* Load description file and train and test trajectories */
		Description description = trajectoryDAO.loadDescription(descriptionPathFile);
		
		List<ITrajectory> train = Utils.loadTrajectories(trainDirPath, description);
		List<ITrajectory> test = Utils.loadTrajectories(testDirPath, description);
				
		if (train.isEmpty()) {
			System.out.println("Empty training set");
			return;
		}
				
		System.out.println("Extracting features from training set...");
		new DodgeFeatures().fillAllTrajectories(train, description);
							
		System.out.println("Writing files in..."+resultDirPath);		
		Utils.writeTrajectoriesToGSON(train, description, resultDirPath + "train.json");		
		Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
		
		System.out.println("Done.");
		
		if (test.isEmpty()){
			return;
		}
		
		System.out.println("Extracting features from test set...");		
		new DodgeFeatures().fillAllTrajectories(test, description);
		
		System.out.println("Writing files i..."+resultDirPath);
		Utils.writeTrajectoriesToGSON(test, description, resultDirPath + "train.json");
		Utils.writeAttributesCSV(test, resultDirPath + "test.csv");
		
		System.out.println("Done.");
				
	}

}
