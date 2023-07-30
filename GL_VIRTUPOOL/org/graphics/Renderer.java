package graphics;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Hashtable;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.border.EmptyBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.FPSAnimator;

import opencv.ObjRecognitionController;

public class Renderer{
   
	static JFrame frame = null;
	private static JSlider slider = new JSlider(JSlider.HORIZONTAL, 90, 270, 90);
	private static JPanel topPanel = new JPanel();
	public static JLabel statusLabel = new JLabel("Calibrating...", JLabel.CENTER);
	
	public static GLU glu = new GLU();
	public static FPSAnimator animator = null;
	public static GLCanvas canvas = null;
	
	public static DiscreteDynamicsWorld dynamicsWorld = null;
	public static RigidBody cueBall = null;
	public static RigidBody[] balls = new RigidBody[15];
	public static RigidBody[] ground = new RigidBody[5];
	
	public static boolean calibrated = false;
	public static boolean calculatedHomography = false;
	public static boolean shot_prediction = true;
	
	public static boolean cueMoving = false;
	public static Point cuePos = null;
	public static Point restPos = null;
	public static boolean stroke = false;
	public static boolean stroke_prepped = false;
	public static int stroke_counter = 7;
	public static float marker_separation_constant = 7;
	
	public static boolean p1turn = true;
	public static JLabel p1scoreLabel = new JLabel();
	public static JLabel p2scoreLabel = new JLabel();
	public static JPanel p1Panel = new JPanel();
	public static JPanel p2Panel = new JPanel();
	
	public static JPanel calibrationPanel = new JPanel();
	
	public static void init () {
		frame = new JFrame( "VirtuPool" );
			//clear frame
		frame.getContentPane().removeAll();
		
		// initialize calibration fields
		initializeCalibration();

		frame.setLocationRelativeTo(null);
	    frame.setExtendedState(JFrame.MAXIMIZED_BOTH); 
	    // frame.setUndecorated(true);
		frame.setVisible( true );
	}
	
	public static void initializeGame() {
		if(calibrated) {
			frame.getContentPane().removeAll();
			
			initializePhysics();
			
			initializeGL();
			//Renderer.cueBall.applyForce(new Vector3f(500, 0, 10), new Vector3f(0,0,0));
			
	//		start the camera capture
		    ObjRecognitionController o = new ObjRecognitionController();
		    o.startCamera();
			
	//		start the GL animation
			animator.start();
			
			//Set up the slider to rotate the image
			initializeSlider();
			
			//set up the top menu
			initializeMenu();
	
			//Add Areas to the content pane
			frame.getContentPane().add( slider, BorderLayout.SOUTH );
			frame.getContentPane().add( topPanel, BorderLayout.NORTH );
			
			frame.getContentPane().revalidate();
			frame.getContentPane().repaint();
		}
	}
	
	private static void initializeCalibration() {
		// TODO Auto-generated method stub
		
		// later do this 
		calibrationPanel.setBackground(new Color(1f, 0f, 1f));
		
		JLabel calibrationInstructions = new JLabel("Hold Pool Cue in the center of the screen and begin calibration", JLabel.CENTER);
		calibrationInstructions.setFont(new Font("Verdana",1,30));
		calibrationInstructions.setBorder(new EmptyBorder(10,10,10,10));
		
		JPanel buttonPanel = new JPanel();
		buttonPanel.setLayout(new GridLayout(1, 3));
		buttonPanel.setBorder(new EmptyBorder(10,10,10,10));
		
		JButton calibrateButton = new JButton("Calibrate");
		calibrateButton.setFont(new Font("Verdana",1,30));
		calibrateButton.addActionListener(new ActionListener()  {
			@Override
			public void actionPerformed(ActionEvent actionEvent) {
				//begin calibration now
				ObjRecognitionController o = new ObjRecognitionController();
				o.startCamera();
			}
		});
		
		buttonPanel.add(new JPanel());
		buttonPanel.add(calibrateButton);
		buttonPanel.add(new JPanel());
		
		frame.getContentPane().add(calibrationPanel, BorderLayout.CENTER);
		frame.getContentPane().add( buttonPanel, BorderLayout.SOUTH );
		frame.getContentPane().add( calibrationInstructions, BorderLayout.NORTH );
		
		
	}

	public static void initializePhysics() {

		float[][] positions = {
			{28, 1f, 0},
			{30.25f, 1f, -1.125f},{30.25f, 1f, 1.125f},
			{32.5f, 1f, 2.25f},{32.5f, 1f, 0f},{32.5f, 1f, -2.25f},
			{34.75f, 1f, 3.375f},{34.75f, 1f, 1.125f},{34.75f, 1f, -1.125f},{34.75f, 1f, -3.375f},
			{37f, 1f, 4.5f},{37f, 1f, 2.25f},{37f, 1f, 0},{37f, 1f, -2.25f},{37f, 1f, -4.5f}
		};
		
		//set up the physics
		BroadphaseInterface broadphase = new DbvtBroadphase();
		DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		buildGround(dynamicsWorld);
		// set the gravity of our world
		dynamicsWorld.setGravity(new Vector3f(0, -10, 0));
		
		// setup our collision shapes
		
		CollisionShape fallShape = new SphereShape(1.125f);
		CollisionShape longWall = new BoxShape(new Vector3f(128f, 10f, 4f));
		CollisionShape shortWall = new BoxShape(new Vector3f(4f, 10f, 68f));
		
		
		DefaultMotionState longWallLeft = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 5, -34), 1.0f))); 
		
		RigidBodyConstructionInfo longWallLeftRigidBodyCI = new RigidBodyConstructionInfo(0, longWallLeft, longWall, new Vector3f(0,0,0)); 
		longWallLeftRigidBodyCI.restitution = 0.25f;
		RigidBody longWallLeftRigidBody = new RigidBody(longWallLeftRigidBodyCI);  
		longWallLeftRigidBody.setFriction(1);
		
		dynamicsWorld.addRigidBody(longWallLeftRigidBody); // add our left long wall to the dynamic world.. 
		
		DefaultMotionState longWallRight = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 5, 34), 1.0f))); 
		
		RigidBodyConstructionInfo longWallRightRigidBodyCI = new RigidBodyConstructionInfo(0, longWallRight, longWall, new Vector3f(0,0,0)); 
		longWallRightRigidBodyCI.restitution = 0.25f;
		RigidBody longWallRightRigidBody = new RigidBody(longWallRightRigidBodyCI); 
		longWallRightRigidBody.setFriction(1);
		
		dynamicsWorld.addRigidBody(longWallRightRigidBody); // add our right long wall to the dynamic world.. 
		
		DefaultMotionState shortWallLeft = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(-64f, 5, 0), 1.0f))); 
		
		RigidBodyConstructionInfo shortWallLeftRigidBodyCI = new RigidBodyConstructionInfo(0, shortWallLeft, shortWall, new Vector3f(0,0,0)); 
		shortWallLeftRigidBodyCI.restitution = 0.25f;
		RigidBody shortWallLeftRigidBody = new RigidBody(shortWallLeftRigidBodyCI); 
		shortWallLeftRigidBody.setFriction(1);
		
		dynamicsWorld.addRigidBody(shortWallLeftRigidBody); // add our Left short wall to the dynamic world..
		
		DefaultMotionState shortWallRight = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(64f, 5, 0), 1.0f))); 
		
		RigidBodyConstructionInfo shortWallRightRigidBodyCI = new RigidBodyConstructionInfo(0, shortWallRight, shortWall, new Vector3f(0,0,0)); 
		shortWallRightRigidBodyCI.restitution = 0.25f;
		shortWallRightRigidBodyCI.angularDamping = 0.95f;
		RigidBody shortWallRightRigidBody = new RigidBody(shortWallRightRigidBodyCI); 
		shortWallRightRigidBody.setFriction(1);
		
		dynamicsWorld.addRigidBody(shortWallRightRigidBody); // add our right short wall to the dynamic world.. 
		
		// setup the motion state for the ball
		DefaultMotionState fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(-28, 1.125f, 0), 1.0f)));
		
		//This we're going to give mass so it responds to gravity 
		float mass = 0.1559f;
		
		Vector3f fallInertia = new Vector3f(0,0,0); 
		fallShape.calculateLocalInertia(mass,fallInertia); 
		
		RigidBodyConstructionInfo fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,fallShape,fallInertia); 
		fallRigidBodyCI.restitution = .5f;
		fallRigidBodyCI.angularDamping = .95f;
		cueBall = new RigidBody(fallRigidBodyCI);  
		cueBall.setFriction(.5f); 

		//now we add it to our physics simulation 
		dynamicsWorld.addRigidBody(cueBall);
		
		//add rest of the balls
		// setup the motion state for the ball
		for(int i = 0; i < 15; i ++) {
				fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(positions[i][0], positions[i][1], positions[i][2]), 1.0f)));
				
				
				fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,fallShape,fallInertia); 
				fallRigidBodyCI.restitution = .5f;
				fallRigidBodyCI.angularDamping = .95f;
				balls[i] = new RigidBody(fallRigidBodyCI); 
				balls[i].setFriction(.5f); 

				//now we add it to our physics simulation 
				dynamicsWorld.addRigidBody(balls[i]);
		}
		
	}

	private static void initializeGL() {
		//set up the GL Area of the frame
		GLProfile.initSingleton();
		GLProfile profile = GLProfile.get(GLProfile.GL2);
		GLCapabilities caps = new GLCapabilities(profile);

		canvas = new GLCanvas(caps);
		GLEventListener listener = new EventListener();
		canvas.addGLEventListener(listener);
		frame.add(canvas);
		animator = new FPSAnimator(canvas, 60);
		frame.addWindowListener( new WindowAdapter() {
			public void windowClosing( WindowEvent windowevent ) {
				frame.remove( canvas );
				frame.dispose();
				System.exit( 0 );
			}
		});
	}
	private static void initializeSlider() {
		slider.addChangeListener(new ChangeListener() {

			@Override
			public void stateChanged(ChangeEvent e) {
				JSlider source = (JSlider)e.getSource();
				EventListener.rotation = (int)source.getValue();
				cuePos = null;
			}
			
		});
		Hashtable<Integer, JLabel> labelTable = new Hashtable<Integer, JLabel>();
		labelTable.put( 90, new JLabel("0 Degrees") );
		labelTable.put( 180, new JLabel("90 Degrees") );
		labelTable.put( 270, new JLabel("180 Degrees") );
		slider.setLabelTable( labelTable );
		slider.setMajorTickSpacing(10);
		slider.setPaintTicks(true);
		slider.setPaintLabels(true);
	}
	
	private static void initializeMenu() {
		topPanel.setLayout(new GridLayout(1, 4));
		
		statusLabel.setFont(new Font("Verdana",1,30));
		topPanel.add(statusLabel, 0);
		
		p1Panel = new JPanel();
		p1Panel.setLayout(new GridLayout(2, 1));
		JLabel p1status = new JLabel("Player 1", JLabel.CENTER);
		p1status.setFont(new Font("Verdana",1,20));
		p1Panel.setOpaque(true);
		p1Panel.setBackground(Color.CYAN);

		p1scoreLabel = new JLabel("7", JLabel.CENTER);
		p1scoreLabel.setFont(new Font("Verdana",1,12));
		p1Panel.add(p1status);
		p1Panel.add(p1scoreLabel);
		topPanel.add(p1Panel, 1);

		p2Panel = new JPanel();
		p2Panel.setLayout(new GridLayout(2, 1));
		JLabel p2status = new JLabel("Player 2", JLabel.CENTER);
		p2status.setFont(new Font("Verdana",1,20));
		p2Panel.setOpaque(true);
		p2Panel.setBackground(Color.WHITE);
		
		p2scoreLabel = new JLabel("7", JLabel.CENTER);
		p2scoreLabel.setFont(new Font("Verdana",1,12));
		p2Panel.add(p2status);
		p2Panel.add(p2scoreLabel);
		topPanel.add(p2Panel, 2);
		
		JButton menuButton = new JButton("Menu");
		menuButton.addActionListener(new ActionListener()  {
			@Override
			public void actionPerformed(ActionEvent actionEvent) {
				JFrame menuFrame = new JFrame( "VirtuPool" );
				//clear frame
				menuFrame.getContentPane().removeAll();
				
				JPanel menuPanel = new JPanel();
				menuPanel.setLayout(new GridLayout(4, 1));
				menuPanel.setBorder(new EmptyBorder(100,100,100,100));
				
				
				String s = "";
				if(Renderer.shot_prediction) {
					s = "Turn Off Shot Tracer";
				}
				else {
					s = "Turn On Shot Tracer";
				}
				JButton guideButton = new JButton(s);
				guideButton.addActionListener(new ActionListener() {

					@Override
					public void actionPerformed(ActionEvent arg0) {
						if(Renderer.shot_prediction) {
							Renderer.shot_prediction = false;
							guideButton.setText("Turn On Shot Tracer");
						}
						else {
							Renderer.shot_prediction = true;
							guideButton.setText("Turn Off Shot Tracer");
						}
					}
					
				});
				JPanel buttonPanel = new JPanel();
				buttonPanel.setLayout(new GridLayout(1, 3));
				buttonPanel.setBorder(new EmptyBorder(50,100,50,100));

				buttonPanel.add(new JPanel());
				buttonPanel.add(guideButton);
				buttonPanel.add(new JPanel());
				menuPanel.add(buttonPanel);
				
				JButton resetButton = new JButton("Restart Game");
				resetButton.addActionListener(new ActionListener() {

					@Override
					public void actionPerformed(ActionEvent arg0) {
						 cuePos = null;
						 cueMoving = false;
						 cuePos = null;
						 restPos = null;
						 stroke = false;
						 stroke_prepped = true;
						 stroke_counter = 0;
						 balls = new RigidBody[15];
						 ground = new RigidBody[5];
						 initializePhysics();
						 menuFrame.dispatchEvent(new WindowEvent(menuFrame, WindowEvent.WINDOW_CLOSING));
					}
					
				});buttonPanel = new JPanel();
				buttonPanel.setLayout(new GridLayout(1, 3));
				buttonPanel.setBorder(new EmptyBorder(50,100,50,100));

				buttonPanel.add(new JPanel());
				buttonPanel.add(resetButton);
				buttonPanel.add(new JPanel());
				menuPanel.add(buttonPanel);
				

				JButton quitButton = new JButton("Quit Game");
				quitButton.addActionListener(new ActionListener() {

					@Override
					public void actionPerformed(ActionEvent arg0) {
						 frame.dispatchEvent(new WindowEvent(menuFrame, WindowEvent.WINDOW_CLOSING));	
						 menuFrame.dispatchEvent(new WindowEvent(menuFrame, WindowEvent.WINDOW_CLOSING));	
					}
					
				});buttonPanel = new JPanel();
				buttonPanel.setLayout(new GridLayout(1, 3));
				buttonPanel.setBorder(new EmptyBorder(50,100,50,100));

				buttonPanel.add(new JPanel());
				buttonPanel.add(quitButton);
				buttonPanel.add(new JPanel());
				menuPanel.add(buttonPanel);
				
				JButton cancelButton = new JButton("Close Menu");
				cancelButton.setPreferredSize(new Dimension(40, 40));
				cancelButton.addActionListener(new ActionListener() {

					@Override
					public void actionPerformed(ActionEvent arg0) {
						 menuFrame.dispatchEvent(new WindowEvent(menuFrame, WindowEvent.WINDOW_CLOSING));	
					}
					
				});buttonPanel = new JPanel();
				buttonPanel.setLayout(new GridLayout(1, 3));
				buttonPanel.setBorder(new EmptyBorder(50,100,50,100));

				buttonPanel.add(new JPanel());
				buttonPanel.add(cancelButton);
				buttonPanel.add(new JPanel());
				menuPanel.add(buttonPanel);
				
				// initialize calibration fields
				menuFrame.getContentPane().add( menuPanel, BorderLayout.CENTER );
	
				menuFrame.setLocationRelativeTo(null);
				menuFrame.setExtendedState(JFrame.MAXIMIZED_BOTH); 
			    menuFrame.setUndecorated(true);
				menuFrame.setVisible( true );
			}
		});
		topPanel.add(menuButton, 3);
		
	}
	
	private static void buildGround(DiscreteDynamicsWorld dynamicsWorld) {
		CollisionShape groundShape = new BoxShape(new Vector3f(60f, 1f, 26f));
		CollisionShape groundSides = new BoxShape(new Vector3f(27f, 1f, 2f));
		
		float[][] origins = {{29, -1, 28}, {29, -1, -28}, {-29, -1, 28}, {-29, -1, -28}};
		
		// setup the motion state
		DefaultMotionState groundMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, -1, 0), 1.0f))); 
		

		// setup the center of the ground
		RigidBodyConstructionInfo groundRigidBodyCI = new RigidBodyConstructionInfo(0, groundMotionState, groundShape, new Vector3f(0,0,0)); 
		
		//restitution (bounciness)
		groundRigidBodyCI.restitution = 0.5f;
		
		// create rigid body to add to world
		RigidBody groundRigidBody = new RigidBody(groundRigidBodyCI);  
		groundRigidBody.setFriction(2); 
		ground[0] = groundRigidBody;
		
		//add rigid body
		dynamicsWorld.addRigidBody(groundRigidBody); // add our ground to the dynamic world.. 
		
		for(int i = 0; i < 4; i++) {
			groundMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(origins[i][0], origins[i][1], origins[i][2]), 1.0f))); 
			

			// setup the center of the ground
			groundRigidBodyCI = new RigidBodyConstructionInfo(0, groundMotionState, groundSides, new Vector3f(0,0,0)); 
			
			//restitution (bounciness)
			groundRigidBodyCI.restitution = 0.5f;
			
			// create rigid body to add to world
			groundRigidBody = new RigidBody(groundRigidBodyCI);  
			groundRigidBody.setFriction(2); 
			ground[i+1] = groundRigidBody;
			
			//add rigid body
			dynamicsWorld.addRigidBody(groundRigidBody); // add our ground to the dynamic world..
		}
	}
	
}
