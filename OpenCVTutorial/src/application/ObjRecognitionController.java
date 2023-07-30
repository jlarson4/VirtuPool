package application;

import java.awt.Point;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;

import application.Utils;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

/**
 * The controller associated with the only view of our application. The
 * application logic is implemented here. It handles the button for
 * starting/stopping the camera, the acquired video stream, the relative
 * controls and the image segmentation process.
 * 
 * @author <a href="mailto:luigi.derussis@polito.it">Luigi De Russis</a>
 * @version 2.0 (2017-03-10)
 * @since 1.0 (2015-01-13)
 * 
 */
public class ObjRecognitionController
{
	// FXML camera button
	@FXML
	private Button cameraButton;
	// the FXML area for showing the current frame
	@FXML
	private ImageView originalFrame;
	// the FXML area for showing the mask
	@FXML
	private ImageView maskImage;
	// the FXML area for showing the output of the morphological operations
	@FXML
	private ImageView morphImage;
	// FXML slider for setting HSV ranges
	@FXML
	private Slider hueStart;
	@FXML
	private Slider hueStop;
	@FXML
	private Slider saturationStart;
	@FXML
	private Slider saturationStop;
	@FXML
	private Slider valueStart;
	@FXML
	private Slider valueStop;
	// FXML label to show the current values set with the sliders
	@FXML
	private Label hsvCurrentValues;

	private Mat H = null;
	
	// a timer for acquiring the video stream
	private ScheduledExecutorService timer;
	// the OpenCV object that performs the video capture
	private VideoCapture capture = new VideoCapture(1);
	// a flag to change the button behavior
	private boolean cameraActive;
	//small distance between markers
	private int small_dist;
	//large distance between markers
	private int large_dist;
	
	private Rect area;
	
	// property for object binding
	private ObjectProperty<String> hsvValuesProp;
		
	/**
	 * The action triggered by pushing the button on the GUI
	 */
	@FXML
	private void startCamera()
	{	
		// bind a text property with the string containing the current range of
		// HSV values for object detection
		hsvValuesProp = new SimpleObjectProperty<>();
		this.hsvCurrentValues.textProperty().bind(hsvValuesProp);
				
		// set a fixed width for all the image to show and preserve image ratio
		this.imageViewProperties(this.originalFrame, 1000);
		this.imageViewProperties(this.maskImage, 400);
		this.imageViewProperties(this.morphImage, 400);
		
		if (!this.cameraActive)
		{
			// start the video capture
			this.capture.open(1);
			
			// is the video stream available?
			if (this.capture.isOpened())
			{
				this.cameraActive = true;
				
				//calibrate the cue markers
				calibrateCue();
				
				// grab a frame every 33 ms (30 frames/sec)
				Runnable frameGrabber = new Runnable() {
					
					@Override
					public void run()
					{
						// effectively grab and process a single frame
						Mat frame = grabFrame();
						// convert and show the frame
						Image imageToShow = Utils.mat2Image(frame);
						updateImageView(originalFrame, imageToShow);
					}
				};
				
				this.timer = Executors.newSingleThreadScheduledExecutor();
				this.timer.scheduleAtFixedRate(frameGrabber, 0, 33, TimeUnit.MILLISECONDS);
				
				// update the button content
				this.cameraButton.setText("Stop Camera");
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
			// update again the button content
			this.cameraButton.setText("Start Camera");
			
			// stop the timer
			this.stopAcquisition();
		}
	}
	
	public void calibrateCue() {
		Mat frame = new Mat();
		
		// check if the capture is open
		if (this.capture.isOpened())
		{
			try
			{
				// read the current frame
				this.capture.read(frame);
				
				// if the frame is not empty, process it
				if (!frame.empty())
				{
					// init
					Mat blurredImage = new Mat();
					Mat hsvImage = new Mat();
					Mat mask = new Mat();
					Mat morphOutput = new Mat();
					
					// remove some noise
					Imgproc.blur(frame, blurredImage, new Size(10, 10));
					
					// convert the frame to HSV
					Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
					

					// get thresholding values from the UI
					// remember: H ranges 0-180, S and V range 0-255
					//using sliders
					Scalar minValues = new Scalar(this.hueStart.getValue(),  75f, 75f); //new Scalar(this.hueStart.getValue(),);
					Scalar maxValues = new Scalar(this.hueStop.getValue(), 255f, 255f); //(this.hueStop.getValue(), this.saturationStop.getValue(), this.valueStop.getValue());
					
					// show the current selected HSV range
					String valuesToPrint = "Hue range: " + minValues.val[0] + "-" + maxValues.val[0]
					+ "\tSaturation range: " + minValues.val[1] + "-" + maxValues.val[1] + "\tValue range: "
					+ minValues.val[2] + "-" + maxValues.val[2];
					Utils.onFXThread(this.hsvValuesProp, valuesToPrint);
					
					// get thresholding values from the UI
					// remember: H ranges 0-180, S and V range 0-255
//					Scalar minValues = new Scalar(129.9f, 0f, 228.0f);
//					Scalar maxValues = new Scalar(156.0f, 47f, 255f);
					
					// threshold HSV image to select tennis balls
					Core.inRange(hsvImage, minValues, maxValues, mask);
					// show the partial output

					this.updateImageView(this.maskImage, Utils.mat2Image(mask));
					
					// morphological operators
					// dilate with large element, erode with small ones
//					Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
					Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));

					Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20, 20));
					
					Imgproc.erode(mask, morphOutput, erodeElement);
					Imgproc.erode(morphOutput, morphOutput, erodeElement);
					Imgproc.erode(morphOutput, morphOutput, erodeElement);
					Imgproc.erode(morphOutput, morphOutput, erodeElement);
					
					Imgproc.dilate(morphOutput, morphOutput, dilateElement);
					Imgproc.dilate(morphOutput, morphOutput, dilateElement);
					
					// show the partial output
					this.updateImageView(this.morphImage, Utils.mat2Image(morphOutput));
					
					// find the tennis ball(s) contours and show them
					this.analyzeCalibration(morphOutput, frame);
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
	
	/**
	 * Get a frame from the opened video stream (if any)
	 * 
	 * @return the {@link Image} to show
	 */
	
	private Mat grabFrame()
	{
		Mat frame = new Mat();
		
		// check if the capture is open
		if (this.capture.isOpened())
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
						frame = this.findMarkers(frame,  frame);
						//frame = this.findAndCircle(frame);
						
					}
					else {
						this.calibrateCue();
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
		
		return frame;
	}
	
	
	private void analyzeCalibration(Mat maskedImage, Mat frame) {
		// init
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		
		// find contours
		Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
		
		// if any contour exist...
		if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
		{

			List<org.opencv.core.Point> sceneList = new LinkedList<org.opencv.core.Point>();
			
			// for each contour, display it in blue
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
					sceneList.add(new org.opencv.core.Point(r.x, r.y));
					sceneList.add(new org.opencv.core.Point(r.x, r.height + r.y));
					sceneList.add(new org.opencv.core.Point(r.width + r.x, r.height + r.y));
					sceneList.add(new org.opencv.core.Point(r.width + r.x,r.y));
					MatOfPoint2f obj = new MatOfPoint2f();
					obj.fromList(sceneList);
					
					H = Calib3d.findHomography(contours_poly, obj);
				}
					
			}
		}
	}

	private Mat findAndDrawCueBall(Mat maskedImage, Mat frame)
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
			
			// for each contour, display it in blue
			for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0])
			{

				
				//get bounding rectangle of each contour
				MatOfPoint2f contours_poly = new MatOfPoint2f();
				MatOfPoint2f  NewMtx = new MatOfPoint2f( contours.get(i).toArray() );
				Imgproc.approxPolyDP(NewMtx, contours_poly, 0.1*Imgproc.arcLength(NewMtx,true), true);
				MatOfPoint  new_poly = new MatOfPoint( contours_poly.toArray() );
				Rect r =  Imgproc.boundingRect(new_poly); 
				
				if(H == null) {
					sceneList.add(new org.opencv.core.Point(r.width + r.x,r.y));
					sceneList.add(new org.opencv.core.Point(r.x, r.y));
					sceneList.add(new org.opencv.core.Point(r.x, r.height + r.y));
					sceneList.add(new org.opencv.core.Point(r.width + r.x, r.height + r.y));
					MatOfPoint2f obj = new MatOfPoint2f();
					obj.fromList(sceneList);
					
					//H = Calib3d.findHomography(contours_poly, obj, Calib3d.LMEDS, 1);
				}
				
				//check rectangle to see if it should be processed
				Moments M = Imgproc.moments(contours.get(i), false);
				int cX = (int) (M.get_m10() / M.get_m00());
				int cY = (int) (M.get_m01() / M.get_m00());
				List<MatOfPoint> temp = new ArrayList<MatOfPoint>();
				//check rectangle to see if it should be processed

				Point temp3 = new Point(cX, cY);
				org.opencv.core.Point temp2 = new org.opencv.core.Point(cX, cY);
				if(this.area.contains(temp2)) {
					System.out.print(temp3.toString() + "\n");

				}
				temp.add(new_poly);
				Imgproc.drawContours(frame, temp, 0, new Scalar(250, 0, 0));
					
			}
		}
		
		return frame;
	}

	private Mat findAndCircle(Mat frame)
	{
		Mat blurredImage = new Mat();
		Mat grayImage = new Mat();
		Mat mask = new Mat();
		
		// remove some noise
		Imgproc.blur(frame, blurredImage, new Size(10, 10));

		this.updateImageView(this.maskImage, Utils.mat2Image(blurredImage));
		Imgproc.cvtColor(frame, grayImage, Imgproc.COLOR_BGR2GRAY);
		Imgproc.GaussianBlur( grayImage, grayImage, new Size(9, 9), 2, 2 );
		Mat detectedCircles = new Mat();
		// show the partial output
	
		this.updateImageView(this.morphImage, Utils.mat2Image(grayImage));
		Scalar minValues = new Scalar(this.hueStart.getValue(), this.saturationStart.getValue(), this.valueStart.getValue());
		Scalar maxValues = new Scalar(this.hueStop.getValue(), this.saturationStop.getValue(), this.valueStop.getValue());
		Imgproc.HoughCircles(grayImage, detectedCircles, Imgproc.HOUGH_GRADIENT,
		          1,   // accumulator resolution (size of the image / 2)
		          (int)this.valueStart.getValue(),  // minimum distance between two circles
		          (int)this.valueStop.getValue(), // Canny high threshold
		          (int)this.saturationStart.getValue(), // minimum number of votes
		          (int)this.hueStart.getValue(), (int)this.hueStop.getValue()); // min and max radius
		

		// show the current selected HSV range
		String valuesToPrint = "Hue range: " + minValues.val[0] + "-" + maxValues.val[0]
		+ "\tSaturation range: " + minValues.val[1] + "-" + maxValues.val[1] + "\tValue range: "
		+ minValues.val[2] + "-" + maxValues.val[2];
		Utils.onFXThread(this.hsvValuesProp, valuesToPrint);
	
		if (detectedCircles != null && !detectedCircles.empty()) {
			for(int i = 0; i < detectedCircles.size().width; i++) {
				double[] vCircle = detectedCircles.get(0, i);
				org.opencv.core.Point pt = new org.opencv.core.Point((int)Math.round(vCircle[0]), (int)Math.round(vCircle[1]));
		        int radius = (int)Math.round(vCircle[2]);
		        if(this.area.contains(pt)) {
			        Imgproc.circle(frame, pt, radius, new Scalar(255, 0, 0), 2);

				}
			}
		}
		
		return frame;
	}

	private Mat findMarkers(Mat maskedImage, Mat frame) {
		// init
		Mat blurredImage = new Mat();
		Mat hsvImage = new Mat();
		Mat mask = new Mat();
		Mat morphOutput = new Mat();
		
		// remove some noise
		Imgproc.blur(frame, blurredImage, new Size(10, 10));
		
		// convert the frame to HSV
		Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		

		// get thresholding values from the UI
		// remember: H ranges 0-180, S and V range 0-255
		//using sliders
		Scalar minValues = new Scalar(this.hueStart.getValue(), 50f, 50f); //new Scalar(this.hueStart.getValue(), this.saturationStart.getValue(), this.valueStart.getValue());
		Scalar maxValues = new Scalar(this.hueStop.getValue(), 255f, 255f); //(this.hueStop.getValue(), this.saturationStop.getValue(), this.valueStop.getValue());
		
		// show the current selected HSV range
		String valuesToPrint = "Hue range: " + minValues.val[0] + "-" + maxValues.val[0]
		+ "\tSaturation range: " + minValues.val[1] + "-" + maxValues.val[1] + "\tValue range: "
		+ minValues.val[2] + "-" + maxValues.val[2];
		Utils.onFXThread(this.hsvValuesProp, valuesToPrint);
		
		// get thresholding values from the UI
		// remember: H ranges 0-180, S and V range 0-255
//		Scalar minValues = new Scalar(129.9f, 0f, 228.0f);
//		Scalar maxValues = new Scalar(156.0f, 47f, 255f);
		
		// threshold HSV image to select tennis balls
		Core.inRange(hsvImage, minValues, maxValues, mask);
		// show the partial output

		this.updateImageView(this.maskImage, Utils.mat2Image(mask));
		
		// morphological operators
		// dilate with large element, erode with small ones
//		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));

		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20, 20));
		
		Imgproc.erode(mask, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		
		// show the partial output
		this.updateImageView(this.morphImage, Utils.mat2Image(morphOutput));
		
		// find the tennis ball(s) contours and show them
		frame = this.findAndDrawCueBall(morphOutput, frame);
		return frame;
	}
	/**
	 * Set typical {@link ImageView} properties: a fixed width and the
	 * information to preserve the original image ration
	 * 
	 * @param image
	 *            the {@link ImageView} to use
	 * @param dimension
	 *            the width of the image to set
	 */
	private void imageViewProperties(ImageView image, int dimension)
	{
		// set a fixed width for the given ImageView
		image.setFitWidth(dimension);
		// preserve the image ratio
		image.setPreserveRatio(true);
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
	 * Update the {@link ImageView} in the JavaFX main thread
	 * 
	 * @param view
	 *            the {@link ImageView} to update
	 * @param image
	 *            the {@link Image} to show
	 */
	private void updateImageView(ImageView view, Image image)
	{
		Utils.onFXThread(view.imageProperty(), image);
	}
	
	/**
	 * On application close, stop the acquisition from the camera
	 */
	protected void setClosed()
	{
		this.stopAcquisition();
	}
	
}