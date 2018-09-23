package br.ufsc.trajectoryclassification.view.xiao.run;

import java.io.File;
import java.text.DecimalFormat;
import java.util.List;

import org.apache.commons.io.FilenameUtils;
import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.xiao.XiaoFeatures;
import br.ufsc.trajectoryclassification.model.dao.ITrajectoryDAO;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.utils.Utils;

public class Xiao {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static int nthreads = 1;
	private static ITrajectoryDAO trajectoryDAO = new TrajectoryDAO();
	private static double[] thresholds = new double[3];
	
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
			case "-tHCR":
				thresholds[0] = Double.valueOf(value);
				break;
			case "-tSR":
				thresholds[1] = Double.valueOf(value);
				break;
			case "-tVCR":
				thresholds[2] = Double.valueOf(value);
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

		str += "Starting running Xiao feature extraction " + System.getProperty("line.separator");

		str += "Configurations:" + System.getProperty("line.separator");

		str += "\tBase directory:	    " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:    " + RESULT_DIR + System.getProperty("line.separator");
		
		str += "\tDescription file :    " + DESCRIPTION_FILE + System.getProperty("line.separator");

		str += "\tAllowed Threads:      " + nthreads + System.getProperty("line.separator");		

		return str;

	}
	
	/*
	private static final double[] thresholds_hurricane_2vs3 = {4.24242400,0.23877920,0.03885423};
	
	private static final double[] thresholds_hurricane_1vs4 = {23.63636000, 0.00552230, 0.08048375};
	
	private static final double[] thresholds_hurricane_0vs45 = {25.45455000, 0.00822163, 0.10435890};
	
	private static final double[] thresholds_animals = {33.33333000, 0.00007627, 0.01340747};
	
	private static final double[] thresholds_vehicle = {21.21212, 0.0917222, 0.06714754};
	*/

	public static void main(String[] args) {

		if (args.length == 0) return;
		/*
		 * STEP 1. Configura parâmetros de entrada
		 */
		configure(args);
		System.out.println(showConfiguration());
		
		if (DESCRIPTION_FILE == null){ 
			System.out.println("We need a description file to understand the input trajectory format.");
			return;
		}
		
		String DESCRIPTION_FILE_NAME = FilenameUtils.removeExtension(
				new File(DESCRIPTION_FILE).getName());
		
		String resultDirPath = RESULT_DIR + "/Xiao/";
		
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
		
		double[] vsize = new double[train.size()];
		for (int i = 0; i < vsize.length; i++) {
			vsize[i] = train.get(i).getData().size();
		}
		
		System.out.println("Extracting features from training set...");
		new XiaoFeatures(thresholds[0], thresholds[1], thresholds[2])
				.fillAllTrajectories(train, description);
							
		System.out.println("Writing files in..."+resultDirPath);		
		Utils.writeTrajectoriesToGSON(train, description, resultDirPath + "train.json");		
		Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
		
		System.out.println("Done.");
		
		if (test.isEmpty()){
			return;
		}
		
		System.out.println("Extracting features from test set...");		
				
		new XiaoFeatures(thresholds[0], thresholds[1], thresholds[2])
					.fillAllTrajectories(test, description);
		
		System.out.println("Writing files i..."+resultDirPath);
		Utils.writeTrajectoriesToGSON(test, description, resultDirPath + "train.json");
		Utils.writeAttributesCSV(test, resultDirPath + "test.csv");
		
		System.out.println("Done.");
				
	}

}
