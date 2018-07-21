package raven.efkslam;

import java.awt.Color;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;

import raven.efkslam.Ellipses;
import raven.efkslam.Landmarks.Landmark;
import raven.efkslam.ConfigFile;
import raven.game.Waypoints;
import raven.game.Waypoints.Wpt;
import raven.math.Vector2D;
import raven.ui.GameCanvas;
import raven.utils.BoundedBuffer;

public class SlamSim implements Runnable {
	private String name;
	private EfkSlamSim efkSlam = new EfkSlamSim();
	Mat wp;
	Mat lm;
	Mat veh;
	Mat xtrue;
	Mat x;
	Mat P;
	Mat ftag;
	Mat da_table;
	Mat QE;
	Mat RE;
	
	double dt;
	double dtsum;
	int iwp; //index to first waypoint (0 based index)
	double G; //initial steer angle
	int iteration;
	
	raven.utils.BoundedBuffer<Pose> buffer;
	Waypoints path = new Waypoints();
	Waypoints trueTrack = new Waypoints();
	Ellipses poseErrorEllipses = new Ellipses();
	
	public SlamSim(String name)
	{
		this.name = name;
		//path = new Waypoints();
		//trueTrack = new Waypoints();
	}
	
	public void initialize(double robot_pos_x, double robot_pos_y)
	{
		veh = new Mat(2, 3, CvType.CV_64F);
		veh.put(0, 0, 0);
		veh.put(0, 1, -ConfigFile.WHEELBASE);
		veh.put(0, 2, -ConfigFile.WHEELBASE);
		veh.put(1, 0, 0);
		veh.put(1, 1, -2);
		veh.put(1, 2, 2);
		
		xtrue = Mat.zeros(3, 1, CvType.CV_64F);
		//initial xtrue = [0]
		//                [0]
		//                [0]
		
		xtrue.put(0, 0, robot_pos_x);
		xtrue.put(1, 0, robot_pos_y);
		
		x = Mat.zeros(3, 1, CvType.CV_64F);
		//initial x = [0] xr
		//            [0] yr
		//            [0] steer angle
		
		x.put(0, 0, robot_pos_x);
		x.put(1, 0, robot_pos_y);
		
		P = Mat.zeros(3, 3, CvType.CV_64F);
		//initial P = [0 0 0]
		//            [0 0 0]
		//            [0 0 0]
		
		dt = ConfigFile.DT_CONTROLS;//change in time between predicts
		dtsum = 0;//change in time since last observation
		
		ftag = new Mat(1, lm.cols(), CvType.CV_64F); // identifier for each landmark
		//initial ftag = [lm1 lm2 lm3 ... lmn]
		
		for(int i = 0; i < lm.cols(); i++)
		{
			ftag.put(0, i, i);
		}
		
		da_table = Mat.zeros(1, lm.cols(), CvType.CV_64F); // data association table
		//initial da_table = [lm1 lm2 lm3 ... lmn]
		
		iwp = 0; //index to first waypoint (0 based index)
		G = 0; //initial steer angle
		QE = ConfigFile.Q(); 
		//initial QE = [SigmaV^2       0]
		//             [0       SigmaG^2]
		
		RE= ConfigFile.R(); 
		//initial RE = [SigmaR^2       0]
		//             [0       SigmaB^2]
		
		if (ConfigFile.SWITCH_INFLATE_NOISE)
		{
			Core.multiply(ConfigFile.Q(), new Scalar(2), QE);
			Core.multiply(ConfigFile.R(), new Scalar(8), RE);
			//inflate estimated noises (ie, add stabilising noise)
		}
		
		iteration = 0;
	}
	
	public void run()
	{
		Vector2D position = new Vector2D();
		int drawPoseEllipseFreqCount = 0;
		while(iwp != -1) // loop iterates over waypoints
		{
			iteration++;
			drawPoseEllipseFreqCount++;
			if(drawPoseEllipseFreqCount > ConfigFile.PoseErrorEllipseFreq) {
				drawPoseEllipseFreqCount = 0;
				Ellipse ellipse = efkSlam.getErrorEllipse(2.4477, new Point(x.get(0, 0)[0], x.get(1, 0)[0]), P.submat(0,2,0,2), xtrue, lm, x, ConfigFile.LogLevel.Off);
				poseErrorEllipses.addEllipse(ellipse);
			}
			//compute true data
			Object[] retValue = efkSlam.compute_steering(xtrue, wp, iwp, ConfigFile.AT_WAYPOINT, G, ConfigFile.RATEG, ConfigFile.MAXG, dt);
			G = (double)retValue[0]; // steer angle
			iwp = (int)retValue[1]; // waypoint index
			
			if (iwp == -1 && ConfigFile.NUMBER_LOOPS > 1) //perform loops: if final waypoint reached, go back to first
			{
				iwp=0; 
				ConfigFile.NUMBER_LOOPS = ConfigFile.NUMBER_LOOPS - 1; 
			}
			
			xtrue = efkSlam.vehicle_model(xtrue, ConfigFile.V, G, ConfigFile.WHEELBASE, dt);
			
			double Vn;
			double Gn;
			double[] retValue1 = efkSlam.add_control_noise(ConfigFile.V, G, ConfigFile.Q(), ConfigFile.SWITCH_CONTROL_NOISE);
			Vn = retValue1[0]; // speed
			Gn = retValue1[1]; // steer angle
			
			//EKF predict step
		    Mat[] retValue2 = efkSlam.predict (x,P, Vn, Gn, QE, ConfigFile.WHEELBASE, dt);
		    x = retValue2[0]; // SLAM state vector, robot pose - position and orientation
		    P = retValue2[1]; // the diagonals of the SLAM covariance matrix
		    
		    //if heading known, observe heading
			Mat[] retValue3 = efkSlam.observe_heading(x, P, xtrue.get(2, 0)[0], ConfigFile.SWITCH_HEADING_KNOWN);
		    x = retValue3[0];
		    P = retValue3[1];
		    
		    //EKF update step
		    dtsum = dtsum + dt;
		    if(dtsum >= ConfigFile.DT_OBSERVE)
		    {
		    	dtsum = 0;
		    	
		    	Mat[] retValue4 = efkSlam.get_observations(xtrue, lm, ftag, ConfigFile.MAX_RANGE);
		    	Mat z = retValue4[0];
		    	Mat ftag_visible = retValue4[1];
		    	
		    	Mat zf = null;
		    	Mat idf = null;
		    	Mat zn = null;
		    	
		    	z = efkSlam.add_observation_noise(z, ConfigFile.R(), ConfigFile.SWITCH_SENSOR_NOISE);
		    	
		    	//compute data association (association not known, estimated using gates)
	    		Mat[] retValue5 = efkSlam.data_associate(x, P, z, RE, ConfigFile.GATE_REJECT, ConfigFile.GATE_AUGMENT); 
	    		zf = retValue5[0];
	    		idf = retValue5[1];
	    		zn = retValue5[2];
		    	
		    	//update step
	    		Mat[] retValue6 = efkSlam.update(x, P, zf, RE, idf); 
	    		x = retValue6[0];
	    		P = retValue6[1];
		    	
	    		
				//augment step
		    	Mat[] retValue7 = efkSlam.augment(x, P, zn, RE); 
		    	x = retValue7[0];
	    		P = retValue7[1];
	    		
	    		position.x = x.get(0, 0)[0];
			    position.y = x.get(1, 0)[0];
			    double trueX = xtrue.get(0, 0)[0];
			    double trueY = xtrue.get(1, 0)[0];
			   
			    trueTrack.addWpt(new Vector2D(trueX, trueY));
			    path.addWpt(new Vector2D(position));
		    }
		}
	}
	
	public Waypoints getSlamPath()
	{
		return this.path;
	}
	public Waypoints getTrueTrack()
	{
		return this.trueTrack;
	}
	public void renderPoseErrorEllipses(){
		poseErrorEllipses.render();
	}
	Ellipse getErrorEllipse(double chisquare, Point mean, Mat covmat, Mat xtrue, Mat lm, Mat x, ConfigFile.LogLevel logLvl) {
		return efkSlam.getErrorEllipse(chisquare, mean, covmat, xtrue, lm, x, logLvl);
	}
	
 	public void setLandmarks(Landmarks landmarks)
	{
		lm = new Mat(2, landmarks.size(), CvType.CV_64F);
		for(int i = 0; i < landmarks.size(); i++)
		{
			Landmark landmark = landmarks.get(i);
			lm.put(0, i, landmark.x);
			lm.put(1, i, landmark.y);
		}
	}
	
	public void setWaypoints(Waypoints waypoints)
	{
		wp = new Mat(2, waypoints.size(), CvType.CV_64F);
		for(int i = 0; i < waypoints.size(); i++)
		{
			Wpt wpt = waypoints.get(i);
			wp.put(0, i, wpt.x);
			wp.put(1, i, wpt.y);
		}
	}
}