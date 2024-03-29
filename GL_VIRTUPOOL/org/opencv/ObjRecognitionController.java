package opencv;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;

import graphics.Renderer;

public class ObjRecognitionController
{
	
	// a timer for acquiring the video stream
	private ScheduledExecutorService timer;
	// the OpenCV object that performs the video capture
	private VideoCapture capture = new VideoCapture(1);
	// a flag to change the button behavior
	private boolean cameraActive;
	//an arraylist of the position of the queue ball across 10 frames with only a single contour
	private List<Point> cueValues = new ArrayList<>();
	private List<Point> restValues = new ArrayList<>();
	
	private List<List<Double>> markerCalibrationPoints = new ArrayList<>();
	
	
	private int missedFrames = 0;
	private int stillFrames = 0;
	private int shotPrepFrames = 0;
	
	private double rest_angle;
	private double rest_dist;
	
	
	// homography matrix
	private Mat H = null;
	
	// rectangle that cue ball must reside within
	private Rect area = null;
		
	/**
	 * The action triggered by pushing the button on the GUI
	 */
	public void startCamera()
	{	
		
		if (!this.cameraActive)
		{
			// start the video capture
			this.capture.open(1);
			
			// is the video stream available?
			if (this.capture.isOpened())
			{
				this.cameraActive = true;
				
				// grab a frame every 33 ms (30 frames/sec)
				Runnable frameGrabber = new Runnable() {
					
					@Override
					public void run()
					{
						// effectively grab and process a single frame
						processFrame();
					}
				};
				
				this.timer = Executors.newSingleThreadScheduledExecutor();
				this.timer.scheduleAtFixedRate(frameGrabber, 0, 66, TimeUnit.MILLISECONDS);
			}
			else
			{
				// log the error
				System.err.println("Failed to open the camera connection...");
			}
		}
		else
		{
			// the camera is not active at this point
			this.cameraActive = false;
			
			// stop the timer
			this.stopAcquisition();
		}
	}
	
	private void calibrateCue(Mat maskedImage) {
		if(markerCalibrationPoints.size() == 10) {
			Renderer.calibrated = true;
			int num = 10;
			float temp = 0;
			for(int i = 0; i < 10; i++) {
				float pos = (float) (markerCalibrationPoints.get(i).get(1)/markerCalibrationPoints.get(i).get(0));

				if(pos > 1.9f && pos < 2.1f) {
					temp += markerCalibrationPoints.get(i).get(0);
				} 
				else {
					num--;
				}
			}
			Renderer.marker_separation_constant = temp / num;
			System.out.print(Renderer.marker_separation_constant);
			setClosed();
			Renderer.initializeGame();
		}
		// init
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		
		// find contours
		Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
		
		// if any contour exist...
		if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
		{
			
			List<Point> marker_points = new ArrayList<>();
			// for each contour, display it in blue

			for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0])
			{
				
				Moments M = Imgproc.moments(contours.get(i), false);
				int cX = (int) (M.get_m10() / M.get_m00());
				int cY = (int) (M.get_m01() / M.get_m00());
				Point temp = new Point(cX, cY);
				org.opencv.core.Point temp2 = new org.opencv.core.Point(cX, cY);
				marker_points.add(temp);
			}
			if(marker_points.size() == 3) {
				List<Double> temp = new ArrayList<>();
				for(int i = 1; i < marker_points.size(); i++) {
					temp.add(marker_points.get(0).distance(marker_points.get(i)));
					System.out.print(temp.toString() + "\n");
				}
				markerCalibrationPoints.add(temp);
			}
		}
	}

	/**
	 * Get a frame from the opened video stream (if any)
	 * 
	 * @return the {@link Image} to show
	 */
	private void processFrame()
	{
		Mat frame = new Mat();
		
		// check if the capture is open and the cue ball is stationary
		if (this.capture.isOpened() && !Renderer.cueMoving)
		{
			try
			{
				// read the current frame
				this.capture.read(frame);
				
				// if the frame is not empty, process it
				if (!frame.empty())
				{
					if(H != null) {
						Mat temp_frame = frame;
						Imgproc.warpPerspective(temp_frame, frame, H, new Size(frame.size().width, frame.size().height));
					}
					Mat blurredImage = new Mat();
					Mat hsvImage = new Mat();
					
					// remove some noise
					Imgproc.blur(frame, blurredImage, new Size(10, 10));
					
					// convert the frame to HSV
					Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
					
					//if the cue is not located and the game has been calibrated look for the cue ball
					if( !Renderer.cueMoving && Renderer.cuePos == null && Renderer.calibrated && Renderer.calculatedHomography) {
						//replace this with circle finder
						analyzeForCueBall(hsvImage);

						if(cueValues.size() >= 10) {

							calculateCuePos();
						}
					}
					
					//if the cue has been located OR the game hasn't been calibrated yet look for markers
					if(Renderer.calculatedHomography && (Renderer.cuePos != null || !Renderer.calibrated)) {
						analyzeForMarkers(hsvImage);
					}
					//if the game has been calibrated, look for homography
					else if(!Renderer.calculatedHomography) {
						findBorders(hsvImage);
					}
				}
			}
			catch (Exception e)
			{
				// log the (full) error
				System.err.print("Exception during the image elaboration...");
				e.printStackTrace();
			}
		}
	}
	
	private void findBorders(Mat hsvImage) {
		// TODO Auto-generated method stub
		// init
		Mat mask = new Mat();
		Mat morphOutput = new Mat();
		
		// get thresholding values from the UI
		// remember: H ranges 0-180, S and V range 0-255
		Scalar minValues = new Scalar(124f, 50.f, 50f);
		Scalar maxValues = new Scalar(180f, 255f, 255f);
		
		// threshold HSV image to select tennis balls
		Core.inRange(hsvImage, minValues, maxValues, mask);
		// show the partial output
		
		// morphological operators
		// dilate with large element, erode with small ones
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20, 20));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
		
		Imgproc.erode(mask, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		
		// show the partial output
		
		// find the cue ball contours and show them
		this.findAndDrawPockets(morphOutput);
	}

	private void calculateCuePos() {
		Point temp = new Point(0, 0);
		for (Point value : cueValues) {
	        temp.x += value.x;
	        temp.y += value.y; 
	    }
		temp.x /= cueValues.size();
		temp.y /= cueValues.size();
		Renderer.cuePos = temp;
		cueValues = new ArrayList<>();
		Renderer.statusLabel.setText("Aim...");
	}

	private void analyzeForCueBall(Mat hsvImage) {
		// TODO Auto-generated method stub
		// init
		Mat mask = new Mat();
		Mat morphOutput = new Mat();
		
		// get thresholding values from the UI
		// remember: H ranges 0-180, S and V range 0-255
		Scalar minValues = new Scalar(109f, 50.f, 50f);
		Scalar maxValues = new Scalar(143f, 255f, 255f);
		
		// threshold HSV image to select tennis balls
		Core.inRange(hsvImage, minValues, maxValues, mask);
		// show the partial output
		
		// morphological operators
		// dilate with large element, erode with small ones
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20, 20));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
		
		Imgproc.erode(mask, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		
		
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		
		// show the partial output
		
		// find the cue ball contours and show them
		this.findAndDrawCueBall(morphOutput);
	}
	
	private void analyzeForMarkers(Mat hsvImage) {
		// init
		Mat mask = new Mat();
		Mat morphOutput = new Mat();
		
		//hardcoded - eventually should be retrieved from the calibrator class
		Scalar minValues = new Scalar(32f, 75f, 75f);
		Scalar maxValues = new Scalar(65f, 255f, 255f);
		
		// threshold HSV image to select tennis balls
		Core.inRange(hsvImage, minValues, maxValues, mask);
		
		// morphological operators
		// dilate with large element
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20, 20));

		Imgproc.erode(mask, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		
		// find the tennis ball(s) contours and show them
		if(Renderer.calibrated) {
			this.findAndDrawBalls(morphOutput);
		}
		else {
			this.calibrateCue(morphOutput);
		}

	}
	
	/**
	 * Given a binary image containing one or more closed surfaces, use it as a
	 * mask to find and highlight the objects contours
	 * 
	 * @param maskedImage
	 *            the binary image to be used as a mask
	 * @param frame
	 *            the original {@link Mat} image to be used for drawing the
	 *            objects contours
	 * @return the {@link Mat} image with the objects contours framed
	 */
	private void findAndDrawBalls(Mat maskedImage)
	{
		// init
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		
		// find contours
		Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
		
		// if any contour exist...
		if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
		{
			ArrayList<Point> marker_points = new ArrayList<>();
			// for each contour, display it in blue
			for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0])
			{

				
				//get bounding rectangle of each contour
				MatOfPoint2f contours_poly = new MatOfPoint2f();
				MatOfPoint2f  NewMtx = new MatOfPoint2f( contours.get(i).toArray() );
				Imgproc.approxPolyDP(NewMtx, contours_poly, 3d, true);
				MatOfPoint  new_poly = new MatOfPoint( contours_poly.toArray() );
				Rect r =  Imgproc.boundingRect(new_poly); 
				
				//check rectangle to see if it should be processed
				if((r.width * r.height) < 10000 && (r.width * r.height) > 500) {
					Moments M = Imgproc.moments(contours.get(i), false);
					int cX = (int) (M.get_m10() / M.get_m00());
					int cY = (int) (M.get_m01() / M.get_m00());
					Point temp = new Point(cX, cY);
					
					//potential markers must be within 5 marker separations of the cue
					if(Renderer.cuePos != null && temp.distance(Renderer.cuePos) < 5*Renderer.marker_separation_constant)
					{
						marker_points.add(temp);
					}
				}
			}
			//check for markers near cue
			if(Renderer.restPos == null && marker_points.size() > 1) {
				Renderer.statusLabel.setText("Aim...");
				Point temp = findClosest(marker_points);
				if(temp != null) {
					restValues.add(temp);
					if(restValues.size() == 15) {
						boolean is_resting = true;
						for(int a = 0; a < restValues.size()-1; a++) {
							for(int b = a+1; b < restValues.size(); b++) {
								if(restValues.get(a).distance(restValues.get(b)) > 50) {
									is_resting = false;
								}
							}
						}
						
						if(is_resting) {
							Renderer.restPos = temp;
							rest_dist = Renderer.restPos.distance(Renderer.cuePos);
							rest_angle = Math.atan2(Renderer.cuePos.y - Renderer.restPos.y, Renderer.cuePos.x - Renderer.restPos.x);
							restValues = new ArrayList<>(); 
						}
						else {
							restValues = new ArrayList<>();
						}
					}
				}

			}
			
			else if(Renderer.restPos != null && marker_points.size() > 1) {
				
				//enough stillFrames, ready to fire
				if(stillFrames >= 20 && !Renderer.stroke) {

					Renderer.statusLabel.setText("Shoot!");
					Renderer.stroke_prepped = true;
					Point p = findClosest(marker_points);
					if(p != null) {
						 if(p.distance(Renderer.cuePos) < Renderer.restPos.distance(Renderer.cuePos)) {
							shotPrepFrames++;
							Renderer.stroke_counter++;
							if(p.distance(Renderer.cuePos) < Renderer.marker_separation_constant/8) {
								//SHOOT
								System.out.print("SHOOTING: " + Renderer.stroke_counter + "\n");
								Renderer.stroke = true;
								stillFrames = 0;
								missedFrames = 0;
								shotPrepFrames = 0;
							} 
						}
							
						 else if(shotPrepFrames >= 40 || (p.distance(Renderer.cuePos) > Renderer.restPos.distance(Renderer.cuePos))) {
							Renderer.stroke_counter = 0;
							stillFrames = 0;
							missedFrames = 0;
							shotPrepFrames = 0;
							Renderer.restPos = null;
							rest_dist = 0;
							rest_angle = 0;
							Renderer.stroke_prepped = false;
							Renderer.statusLabel.setText("Aim...");
						 }
						 else {
							Renderer.stroke_counter = 0;
							shotPrepFrames++;
						 }
					}
				} 

				//compare markers near cue to successive frames
				else if (Renderer.cuePos != null){
					boolean success = false;
					Point closest = findClosest(marker_points);
					double closest_dist = closest.distance(Renderer.cuePos);
					double closest_angle = Math.atan2(Renderer.cuePos.y - Renderer.restPos.y, Renderer.cuePos.x - Renderer.restPos.x);
					if((closest_dist - rest_dist) < (Renderer.marker_separation_constant/8) && (Math.abs(closest_angle - rest_angle) < 1)) {
						success = true;
					}
					if(success) {
						stillFrames++;
					}
					else {
						missedFrames++;
					}
					if(missedFrames > 3) {
						stillFrames = 0;
						missedFrames = 0;
						Renderer.restPos = null;
						rest_dist = 0;
						rest_angle = 0;
						shotPrepFrames=0;
					}
				}
			}
			else {
				missedFrames++;
				if(missedFrames > 3) {
					stillFrames = 0;
					missedFrames = 0;
					Renderer.restPos = null;
					rest_dist = 0;
					rest_angle = 0;
					shotPrepFrames=0;
					Renderer.stroke_prepped = false;
					Renderer.statusLabel.setText("Aim...");
				}
			}
		}
	}

	private Point findClosest(ArrayList<Point> marker_points) {
		boolean success = true;
		int closest = -1;
		float distance = 10000f;
		for(int x = 0; x < marker_points.size(); x++) {
			for(int i = x+1; i < marker_points.size(); i++) {
				if(marker_points.get(x).distance(marker_points.get(i)) > (Renderer.marker_separation_constant * 2) + (Renderer.marker_separation_constant/3)) {
					success = false;
					break;
				}
			}
			if(marker_points.get(x).distance(Renderer.cuePos) < distance) {
				distance = (float) marker_points.get(x).distance(Renderer.cuePos);
				closest = x;
			}
			if(!success) {
				break;
			}
		}
		if(success && closest != -1) {
			return marker_points.get(closest);
		} 
		else {
			return null;
		}
	}

	private void findAndDrawPockets(Mat maskedImage)
	{
		// init
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		
		// find contours
		Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
		
		// if any contour exist...
		if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
		{


			List<org.opencv.core.Point> sceneList = new LinkedList<org.opencv.core.Point>();
			
			for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0])
			{
	
				//get bounding rectangle of each contour
				MatOfPoint2f contours_poly = new MatOfPoint2f();
				MatOfPoint2f  NewMtx = new MatOfPoint2f( contours.get(i).toArray() );
				Imgproc.approxPolyDP(NewMtx, contours_poly, 0.1*Imgproc.arcLength(NewMtx,true), true);
				MatOfPoint  new_poly = new MatOfPoint( contours_poly.toArray() );
				Rect r =  Imgproc.boundingRect(new_poly); 
				this.area = r;

				
				if(H == null) {
					sceneList.add(new org.opencv.core.Point(r.width + r.x,r.y));
					sceneList.add(new org.opencv.core.Point(r.x, r.y));
					sceneList.add(new org.opencv.core.Point(r.x, r.height + r.y));
					sceneList.add(new org.opencv.core.Point(r.width + r.x, r.height + r.y));
					MatOfPoint2f obj = new MatOfPoint2f();
					obj.fromList(sceneList);
					
					H = Calib3d.findHomography(contours_poly, obj, Calib3d.LMEDS, 1);
					Renderer.calculatedHomography = true;
					Renderer.calibrationPanel.setBackground(new Color(0.7f, 0.7f, 0.7f));
				}
				
			}
		}
	}

	private void findAndDrawCueBall(Mat maskedImage)
	{
		// init
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		
		// find contours
		Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
		
		// if any contour exist...
		if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
		{
			
			for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0])
			{
				Moments M1 = Imgproc.moments(contours.get(i), false);
				int cX = (int) (M1.get_m10() / M1.get_m00());
				int cY = (int) (M1.get_m01() / M1.get_m00());
				Point temp = new Point(cX, cY);
				org.opencv.core.Point temp2 = new org.opencv.core.Point(cX, cY);
				if(Renderer.cuePos == null) {
					Renderer.cuePos = temp;
				}
			}
		}
	}
	
	/**
	 * Stop the acquisition from the camera and release all the resources
	 */
	private void stopAcquisition()
	{
		if (this.timer!=null && !this.timer.isShutdown())
		{
			try
			{
				// stop the timer
				this.timer.shutdown();
				this.timer.awaitTermination(33, TimeUnit.MILLISECONDS);
			}
			catch (InterruptedException e)
			{
				// log any exception
				System.err.println("Exception in stopping the frame capture, trying to release the camera now... " + e);
			}
		}
		
		if (this.capture.isOpened())
		{
			// release the camera
			this.capture.release();
		}
	}
	
	/**
	 * On application close, stop the acquisition from the camera
	 */
	protected void setClosed()
	{
		this.stopAcquisition();
	}
	
}