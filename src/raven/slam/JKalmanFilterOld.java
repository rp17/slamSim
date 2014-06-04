package raven.slam;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import raven.math.Vector2D;

public class JKalmanFilterOld {

    /** number of measurement vector dimensions */
    int mp;
    /** number of state vector dimensions */
    int dp;
    /** number of control vector dimensions */
    int cp;
    
    /** predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k) */
    Mat state_pre;           
    /** corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) */
    Mat state_post;
    /** state transition matrix (A) */
    Mat transition_matrix = new Mat();
    /** control matrix (B) (it is not used if there is no control)*/
    Mat control_matrix;
    /** measurement matrix (H) */    
    Mat measurement_matrix;
    /** process noise covariance matrix (Q) */
    Mat process_noise_cov;
    /** measurement noise covariance matrix (R) */
    Mat measurement_noise_cov;
    /** priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q) */
    Mat error_cov_pre;
    /** Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R) */
    Mat gain;
    /** posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) */
    Mat error_cov_post;

    /** temporary matrices */
    Mat temp1;
    Mat temp2;
    Mat temp3;
    Mat temp4;
    Mat temp5;
    Mat temp6;
    Mat temp7;

    public JKalmanFilterOld(int dynam_params, int measure_params, int control_params) 
                throws Exception {
        
        if( dynam_params <= 0 || measure_params <= 0 ) {
            throw new IllegalArgumentException("Kalman filter: Illegal dimensions.");
        }
        
        if( control_params < 0 ) {
            control_params = dynam_params;
        }
        
        // init
        dp = dynam_params;
        mp = measure_params;
        cp = control_params;

        state_pre = new Mat(dp, 1, CvType.CV_64F); // init by transition _matrix*state_post

        // following variables must be initialized properly in advance        
        state_post = new Mat(dp, 1, CvType.CV_64F); // init by the first measurement!!!
        transition_matrix = Mat.eye(dp, dp, CvType.CV_64F); // or init the matrix as:
/*      double[][] tr = { {1, 0, 1, 0},              // { {1, 1},   // x
                           {0, 1, 0, 1},             //   {0, 1} }; // dx
                           {0, 0, 1, 0}, 
                           {0, 0, 0, 1} };
        kalman.transition_matrix = new Matrix(tr);
*/    
        process_noise_cov = Mat.eye(dp, dp, CvType.CV_64F); // 1e-5 (1 in OpenCV)
        Core.multiply(process_noise_cov, new Scalar(0.00001), process_noise_cov);

        measurement_matrix = Mat.eye(mp, dp, CvType.CV_64F); // 1 (0 in OpenCV)
        measurement_noise_cov = Mat.eye(mp, mp, CvType.CV_64F); // 1e-1 (1 in OpenCV)
        Core.multiply(measurement_noise_cov, new Scalar(0.1), measurement_noise_cov);

        error_cov_pre = new Mat(dp, dp, CvType.CV_64F); // initialized in Predict
        error_cov_post = Mat.eye(dp, dp, CvType.CV_64F); // 1 (0 in OpenCV)

        gain = new Mat(dp, mp, CvType.CV_64F);

        if( cp > 0 )
        {
            control_matrix = new Mat(dp, cp, CvType.CV_64F);
        }
        else {
            control_matrix = null;
        }

        temp1 = new Mat(dp, dp, CvType.CV_64F);
        temp2 = new Mat(mp, dp, CvType.CV_64F);
        temp3 = new Mat(mp, mp, CvType.CV_64F);
        temp4 = new Mat(mp, dp, CvType.CV_64F);
        temp5 = new Mat(mp, 1, CvType.CV_64F);   
        temp6 = new Mat(dp, mp, CvType.CV_64F); 
        temp7 = new Mat(dp, 1, CvType.CV_64F);   
    }

    /**
     * Constructor in case of no control.
     * @param dynam_params
     * @param measure_params
     */
    public JKalmanFilterOld(int dynam_params, int measure_params) throws Exception {
        this(dynam_params, measure_params, 0);
    }

    
    /**
     * Alias for prediction with no control.
     * @return Predict(no control).
     */
    public Mat Predict(Vector2D velocity) {
        return Predict(null, velocity);
    }
    
    /**
     * Estimates subsequent model state.
     * <p> The function estimates the subsequent 
     * stochastic model state by its current state and stores it at 
     * <code>state_pre</code>:
     * <pre>
     * x'<sub>k</sub>=A*x<sub>k</sub>+B*u<sub>k</sub>
     * P'<sub>k</sub>=A*P<sub>k-1</sub>*A<sup>T</sup> + Q,
     * where
     * x'<sub>k</sub> is predicted state (state_pre),
     * x<sub>k-1</sub> is corrected state on the previous step (state_post)
     *     (should be initialized somehow in the beginning, zero vector by default),
     * u<sub>k</sub> is external control (<code>control</code> parameter),
     * P'<sub>k</sub> is prior error covariance matrix (error_cov_pre)
     * P<sub>k-1</sub> is posteriori error covariance matrix on the previous step (error_cov_post)
     *     (should be initialized somehow in the beginning, identity matrix by default),
     * </pre>
     * @param control Control vector (u<sub>k</sub>), should be NULL if there 
     * is no external control (<code>control_params</code>=0).
     * @return The function returns the estimated state.
     */
    public Mat Predict(Mat control, Vector2D velocity) {

    	// Prediction steps:
    	// (1) Project the state ahead: x'(k) = A*x(k-1) + B*u(k)
    	// (2) Project the error covariance ahead: P'(k) = A*P(k-1)*At + Q
    	
        // (1) Project the state ahead
    	// u(k) is unknown, so we reduce the equation to x'(k) = A*x(k-1)
    	// A = transition_matrix
    	// x(k-1) = state_post, previous measurement
    	Core.gemm(transition_matrix, state_post, 1, new Mat(), 0, state_pre);
    	
    	Mat velocityMat = new Mat(dp, 1, CvType.CV_64F);
    	velocityMat.put(0, 0, velocity.x);
    	velocityMat.put(1, 0, velocity.y);
    	
    	for(int i = 2; i < dp; i++){
    		velocityMat.put(i, 0, 0);
    	}
    	
    	// update with velocity
    	Core.add(state_pre, velocityMat, state_pre);
    	
    	//if there's control matrix
        if( control != null && cp > 0 ) {
             // x'(k) = x'(k) + B*u(k)
             Core.add(control_matrix.mul(control), state_pre, state_pre);	 
        }

        // (2) Project the error covariance ahead
        // update error covariance matrices: temp1 = A*P(k)
        temp1 = transition_matrix.mul(error_cov_post);
        // P'(k) = temp1*At + Q
        error_cov_pre = temp1.mul(transition_matrix.t());// process_noise_cov, 1, 1);
        Core.add(error_cov_pre, process_noise_cov, error_cov_pre);

        System.out.println("Predict: " + state_pre.dump());
        return state_pre;
    }
    
    /**
     * Adjusts model state.
     * The function <code>KalmanCorrect</code> adjusts stochastic model state 
     * on the basis of the given measurement of the model state:</p>
     * <pre>
     * K<sub>k</sub>=P'<sub>k</sub>*H<sup>T</sup>*(H*P'<sub>k</sub>*H<sup>T</sup>+R)<sup>-1</sup>
     * x<sub>k</sub>=x'<sub>k</sub>+K<sub>k</sub>*(z<sub>k</sub>-H*x'<sub>k</sub>)
     * P<sub>k</sub>=(I-K<sub>k</sub>*H)*P'<sub>k</sub>
     * where
     * z<sub>k</sub> - given measurement (<code>mesurement</code> parameter)
     * K<sub>k</sub> - JKalman "gain" matrix.
     * </pre>
     * <p>The function stores adjusted state at <code>state_post</code> and 
     * returns it on output.
     * @param measurement Matrix containing the measurement vector.
     * @return
     */
    public Mat Correct(Mat measurement) {
        // (1) Compute the Kalman gain
        // temp2 = H*P'(k)
        Core.gemm(measurement_matrix, error_cov_pre, 1, new Mat(), 0, temp2);

        // temp3 = H*P'(k)*Ht + R 
        Core.gemm(temp2,measurement_matrix.t(), 1, measurement_noise_cov, 1, temp3);

        // temp6 = P'(k)*Ht
        Core.gemm(error_cov_pre, measurement_matrix.t(), 1, new Mat(), 0, temp6);
        // K(k) = P'(k)*Ht*(H * P'(k) * Ht + R)-1 
        Core.gemm(temp6, temp3.inv(), 1, new Mat(), 0, temp6);

        // K(k) 
        gain = temp6;
        System.out.println("Gain: " + gain.dump());
        
        System.out.println("Measurement: " + measurement.dump());
        System.out.println("Measurement Matrix: " + measurement_matrix.dump());
        
        // (2) Update estimate with measurement z(k)
        // temp5 = H*x'(k) 
        Core.gemm(measurement_matrix, state_pre, 1, new Mat(), 0, temp5);
        System.out.println("H*x'(k): " + temp5.dump());
        // temp5 = z(k) - H*x'(k) 
        Core.subtract(measurement.t(), temp5, temp5);
        System.out.println("z(k) - H*x'(k): " + temp5.dump());
        
        // temp7 = K(k)*(z(k) - H*x'(k))
        Core.gemm(gain, temp5, 1, new Mat(), 0, temp7);
        System.out.println("K(k)*(z(k) - H*x'(k)): " + temp7.dump());
        // x(k) = x'(k) + K(k)*(z(k) - H*x'(k))
        Core.add(state_pre, temp7, temp7);
        System.out.println("x(k) = x'(k) + K(k)*(z(k) - H*x'(k)): " + temp7.dump());
        
        state_post = temp7;
        System.out.println("State post: " + state_post.dump());

        // (3) Update the error covariance.
        // P(k) = P'(k) - K(k)*H*P'(k)
        Core.gemm(gain, temp2, 1, new Mat(), 0, error_cov_post);
        Core.subtract(error_cov_pre, error_cov_post, error_cov_post);
        System.out.println("error_cov_poast: " + error_cov_post.dump());

        System.out.println("Correct: " + state_post.dump());
        return state_post;
    }

    /**
     * Setter
     * @param state_pre
     */
    public void setState_pre(Mat state_pre) {
        this.state_pre = state_pre;
    }

    /**
     * Getter
     * @return
     */
    public Mat getState_pre() {
        return state_pre;
    }

    /**
     * Setter
     * @param state_post
     */
    public void setState_post(Mat state_post) {
        this.state_post = state_post;
    }

    public Mat getState_post() {
        return state_post;
    }

    /**
     * Getter
     * @param transition_matrix
     */
    public void setTransition_matrix(Mat transition_matrix) {
        this.transition_matrix = transition_matrix;
    }

    public Mat getTransition_matrix() {
        return transition_matrix;
    }

    /**
     * Setter
     * @param control_matrix
     */
    public void setControl_matrix(Mat control_matrix) {
        this.control_matrix = control_matrix;
    }

    /**
     * Getter
     * @return
     */
    public Mat getControl_matrix() {
        return control_matrix;
    }

    /**
     * Setter
     * @param measurement_matrix
     */
    public void setMeasurement_matrix(Mat measurement_matrix) {
        this.measurement_matrix = measurement_matrix;
    }

    /**
     * Getter
     * @return
     */
    public Mat getMeasurement_matrix() {
        return measurement_matrix;
    }

    /**
     * Setter
     * @param process_noise_cov
     */
    public void setProcess_noise_cov(Mat process_noise_cov) {
        this.process_noise_cov = process_noise_cov;
    }

    /**
     * Getter
     * @return
     */
    public Mat getProcess_noise_cov() {
        return process_noise_cov;
    }

    /**
     * Setter
     * @param measurement_noise_cov
     */
    public void setMeasurement_noise_cov(Mat measurement_noise_cov) {
        this.measurement_noise_cov = measurement_noise_cov;
    }

    /**
     * Getter
     * @return
     */
    public Mat getMeasurement_noise_cov() {
        return measurement_noise_cov;
    }

    /**
     * Setter
     * @param error_cov_pre
     */
    public void setError_cov_pre(Mat error_cov_pre) {
        this.error_cov_pre = error_cov_pre;
    }

    /**
     * Getter
     * @return
     */
    public Mat getError_cov_pre() {
        return error_cov_pre;
    }

    /**
     * Setter
     * @param gain
     */
    public void setGain(Mat gain) {
        this.gain = gain;
    }

    /**
     * Getter
     * @return
     */
    public Mat getGain() {
        return gain;
    }

    /**
     * Setter
     * @param error_cov_post
     */
    public void setError_cov_post(Mat error_cov_post) {
        this.error_cov_post = error_cov_post;
    }

    /**
     * Getter
     * @return
     */
    public Mat getError_cov_post() {
        return error_cov_post;
    }
}

