package graphics;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.util.ArrayList;

import javax.vecmath.Point2f;
import javax.vecmath.Vector3f;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.glu.GLUquadric;

public class EventListener implements GLEventListener{

	private static float DEG2RAD = 3.14159f/180f;
	private static final float RAIL_HEIGHT = 1.40625f;
	private static final float LIGHT_POSITION = 30;
	private static final float CUE_MASS = 567f; //in grams
	private static final float MARKER_SPACING = 0.2f; //in meters
	private static Point2f cuePrevious = new Point2f();
	
	public static int rotation = 90;
	
	private static boolean path_predicted = false;
	private static ArrayList<Vector3f> shot_path = new ArrayList<>();
	private static ArrayList<ArrayList<Vector3f>> ball_shot_path = new ArrayList<>(15);
	private static boolean haveMoved[] = new boolean[15]; 
	private static DiscreteDynamicsWorld tempWorld = null;
	
	private static String p1Type = "";
	private static String p2Type = "";
	private static int p1Score = 7;
	private static int p2Score = 7;
	private static boolean[] ball_gone = new boolean[15];
	
	private static int handicap = 8;
	
	private static final float[][] COLORS = {
			{1f, 0f, 0f}, 
			{0.556863f, 0.137255f, 0.137255f}, {1f, 0f, 0f},
			{0.556863f, 0.137255f, 0.137255f}, {0.2f, 0.2f, 0.2f}, {1f, 0f, 0f},
			{1f, 0f, 0f}, {0.556863f, 0.137255f, 0.137255f}, {1f, 0f, 0f}, {0.556863f, 0.137255f, 0.137255f},
			{0.556863f, 0.137255f, 0.137255f}, {1f, 0f, 0f}, {0.556863f, 0.137255f, 0.137255f}, {1f, 0f, 0f}, {0.556863f, 0.137255f, 0.137255f}
	};
	private static final String[] balls_type = {
				"Solids", 
				"Stripes", "Solids",
				"Stripes", "Eight", "Solids",
				"Solids", "Stripes", "Solids", "Stripes",
				"Stripes", "Solids", "Stripes", "Solids", "Stripes"
	};
	
	@Override
	public void display(GLAutoDrawable drawable) {
		
		if(Renderer.stroke) {
			
			double distance = (Renderer.restPos.distance(Renderer.cuePos)/Renderer.marker_separation_constant) * MARKER_SPACING;
			double time = Renderer.stroke_counter / (60d * handicap);
			double force = (distance / time) * CUE_MASS;
			double angle = Math.atan2(Renderer.cuePos.y - Renderer.restPos.y, Renderer.cuePos.x - Renderer.restPos.x) + Math.toRadians(rotation-90f);
			System.out.print(distance + "\n");
			float force_x = (float)(force * Math.cos(angle));
			float force_z = (float)(force * Math.sin(angle));
			System.out.print("X FORCE: " + force_x + " Z FORCE: " + force_z + " ANGLE:" + angle + "\n");
			Renderer.cueBall.activate();
			Renderer.cueBall.applyForce(new Vector3f(force_x, 0, force_z), new Vector3f(0,0,0));
			Renderer.statusLabel.setText("Balls in motion...");
			Renderer.cuePos = null;
			Renderer.restPos = null;
			Renderer.stroke = false;
			Renderer.cueMoving = true;
			Renderer.stroke_counter = 0;
			Renderer.stroke_prepped = false;
			path_predicted = false;
			shot_path = new ArrayList<>();
			ball_shot_path = new ArrayList<>(15);
			haveMoved = new boolean[15];
			
			Renderer.p1turn = !Renderer.p1turn;
			if(Renderer.p1turn) {
				Renderer.p1Panel.setBackground(Color.CYAN);
				Renderer.p2Panel.setBackground(Color.WHITE);
			}
			else {
				Renderer.p1Panel.setBackground(Color.WHITE);
				Renderer.p2Panel.setBackground(Color.CYAN);
			}
		}
		
		if(Renderer.shot_prediction) {
			if(Renderer.stroke_prepped && !path_predicted) {
				while(ball_shot_path.size() < 15) {
					ball_shot_path.add(new ArrayList<Vector3f>());
				}
				buildTempWorld();
				path_predicted = true;
			} else if(!Renderer.stroke_prepped) {
				path_predicted = false;
				shot_path = new ArrayList<>();
				ball_shot_path = new ArrayList<>(15);
				haveMoved = new boolean[15];
			}
		}
		else if(!Renderer.shot_prediction && shot_path.size() > 0) {
	    		shot_path = new ArrayList<>();
	    		ball_shot_path = new ArrayList<>();
	    		haveMoved = new boolean[15];
	    		path_predicted = false;
	    }
		
		Renderer.dynamicsWorld.stepSimulation(1/60.f, 10); 
		
		GL2 gl = drawable.getGL().getGL2();
		setCamera(gl, Renderer.glu, 100);
		gl.glLoadIdentity();//important
		
		//setup lighting function
		setLighting(gl);
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT);
		

	    Transform trans = new Transform();
	    Renderer.ground[0].getMotionState().getWorldTransform(trans);
		
        //playing area
        drawCube(gl, trans.origin.x + 60, trans.origin.y + 1, trans.origin.z + 26, 120, 2, 52);
        
        for(int i = 1; i < 5; i++) {

    	    		Renderer.ground[i].getMotionState().getWorldTransform(trans);
    	        drawCube(gl, trans.origin.x + 27, trans.origin.y + 1, trans.origin.z +2, 54, 2, 4);
        }
        

        setColor(gl, new float[] {1f, 1f, 1f});
        drawCube(gl, 64, -1, 34, 128, 0, 68);
        
        
        	//sides
		
		// build the sides
        setColor(gl, new float[] {0.36f, 0.25f, 0.20f});
        drawCube(gl, 64, 1, 34, 128, RAIL_HEIGHT, 4);
        drawCube(gl, 64, 1, -30, 128, RAIL_HEIGHT, 4);
        drawCube(gl, 64, 1, 30, 4, RAIL_HEIGHT, 60);
        drawCube(gl, -60, 1, 30, 4, RAIL_HEIGHT, 60);
        
        if(Renderer.shot_prediction && path_predicted) {
            setColor(gl, new float[] {1f, 1f, 1f});
	        	gl.glBegin (GL2.GL_LINE_STRIP);
	           for(int i = 0; i < shot_path.size(); i++) {
	        	   		gl.glVertex3f(shot_path.get(i).x, shot_path.get(i).y, shot_path.get(i).z);
	           }
            gl.glEnd();
            
            for(int i = 0; i < haveMoved.length; i++) {
            		if(haveMoved[i]) {
            		 	gl.glBegin (GL2.GL_LINE_STRIP);
         	           for(int x = 0; x < ball_shot_path.get(i).size(); x++) {
         	        	   		gl.glVertex3f(ball_shot_path.get(i).get(x).x, ball_shot_path.get(i).get(x).y, ball_shot_path.get(i).get(x).z);
         	           }
                     gl.glEnd();
            		}
            }
        }
        
        
        setColor(gl, new float[] {0.137255f, 0.137255f, 0.556863f});
        
        //set up sphere
        GLUquadric earth = Renderer.glu.gluNewQuadric();
        Renderer.glu.gluQuadricDrawStyle(earth, GLU.GLU_FILL);
        Renderer.glu.gluQuadricNormals(earth, GLU.GLU_FLAT);
        Renderer.glu.gluQuadricOrientation(earth, GLU.GLU_OUTSIDE);
        final float radius = 1.125f;
        final int slices = 150;
        final int stacks = 150;
        
        //cue ball translate
	    Renderer.cueBall.getMotionState().getWorldTransform(trans);
	    
	    //check if cue ball has fallen off the page
        if (trans.origin.y < -2) {
			Renderer.cueBall.translate(new Vector3f(-28 + (0 - trans.origin.x), 1.125f + (0 - trans.origin.y), 0 + (0 - trans.origin.z)));
	    		Renderer.cueBall.clearForces();
	    		Renderer.cueMoving = false;
	    }
        
        //check position if not fallen
	    if(cuePrevious.x == trans.origin.x && cuePrevious.y == trans.origin.z) {
	    		Renderer.cueMoving = false;
	    } 
	    else {
	    		cuePrevious = new Point2f(trans.origin.x, trans.origin.z);
	    		Renderer.cueMoving = true;
	    }

	    if(!Renderer.cueMoving && Renderer.cuePos == null) {
		    Vector3f[] circleVertices = new Vector3f[37];
		    for (int a = 0; a <= 360; a += 360 / 36)
		    {
		    		
		    		float heading = a * DEG2RAD;
		    		circleVertices[a / (360/ 36)] = new Vector3f((float)Math.cos(heading) * 5.125f, trans.origin.y + 2, (float)Math.sin(heading) * 5.125f);
		    }
		    
		    gl.glBegin(GL.GL_TRIANGLE_FAN);
				gl.glNormal3f(0f, 1f, 0f);
		    		gl.glVertex3f(trans.origin.x, trans.origin.y + 2, trans.origin.z);
				for (int i = 0; i < circleVertices.length; i++) {
					gl.glNormal3f(0f, 1f, 0f);
					gl.glVertex3f(circleVertices[i].x + trans.origin.x, circleVertices[i].y, circleVertices[i].z + trans.origin.z);
				}
			gl.glEnd();
	    }
        setColor(gl, new float[] {1f, 1f, 1f});
        positionBall(gl, earth, radius, slices, stacks, trans.origin.x, trans.origin.y, trans.origin.z);
        
        for(int i = 0; i < 15; i++) {
        		Renderer.balls[i].getMotionState().getWorldTransform(trans);
        		
        		if (trans.origin.y < -4 && !ball_gone[i]) {
        			ball_gone[i] = true;
        			if(balls_type[i] == "Eight") {
        				endGame(true);
        			}
        			if(p1Type == "") {
        				if(Renderer.p1turn) {
        					p1Type = balls_type[i];
        					if(balls_type[i] == "Solids") {
        						p2Type = "Stripes";
        					}
        					else {
        						p2Type = "Solids";
        					}
        				}
        				else {
        					p2Type = balls_type[i];
        					if(balls_type[i] == "Solids") {
        						p1Type = "Stripes";
        					}
        					else {
        						p1Type = "Solids";
        					}
        				}
        			}
        			else {
        				if(p1Type == balls_type[i]) {
        					p1Score--;
        					if(p1Score > 0) {
        						Renderer.p1scoreLabel.setText(Integer.toString(p1Score));
        					}
        					else {
        						Renderer.p1scoreLabel.setText("8 Ball");
        					}
        				}
        				else {
        					p2Score--;
        					if(p2Score > 0) {
        						Renderer.p2scoreLabel.setText(Integer.toString(p2Score));
        					}
        					else {
        						Renderer.p2scoreLabel.setText("8 Ball");
        					}
        				}
        			}
        	    }
        		else {
                 setColor(gl, new float[] {COLORS[i][0], COLORS[i][1], COLORS[i][2]});
                 positionBall(gl, earth, radius, slices, stacks, trans.origin.x, trans.origin.y, trans.origin.z);
        		}
        }
        
        Renderer.glu.gluDeleteQuadric(earth);
	}
	
	@Override
	public void dispose(GLAutoDrawable drawable) {
		// TODO Auto-generated method stub
		Renderer.animator.stop();
		System.exit(0);
	}
	
	@Override
	public void init(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
        gl.glShadeModel(GL2.GL_SMOOTH);
		
		gl.glClearColor(0, 0, 0, 1);
		gl.glClearDepthf(1.0f);
		gl.glEnable(GL2.GL_DEPTH_TEST);
        gl.glDepthFunc(GL2.GL_LEQUAL);
        gl.glHint(GL2.GL_PERSPECTIVE_CORRECTION_HINT, GL2.GL_NICEST);	
	}
	
	@Override
	public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {

		
	}
	
	private void setCamera(GL2 gl, GLU glu, float distance) {
        // Change to projection matrix.
        gl.glMatrixMode(GL2.GL_PROJECTION);
        gl.glLoadIdentity();
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        float width = (float) screenSize.getWidth();
        float height = (float) screenSize.getHeight();
        // Perspective.
        float widthHeightRatio = width / height;
        glu.gluPerspective(45, widthHeightRatio, 1, 1000);
        glu.gluLookAt(0, distance, 0, 0, 0, 0, 1, 0, 0);
        gl.glRotatef(-rotation, 0.0f, 1.0f, 0.0f);
        // Change back to model view matrix.
        gl.glMatrixMode(GL2.GL_MODELVIEW);
        gl.glLoadIdentity();
    }
	
	private void positionBall(GL2 gl, GLUquadric earth, float radius, int slices, int stacks, float x, float y, float z) {
		 gl.glPushMatrix();
	        gl.glTranslatef(x, y, z);
	        Renderer.glu.gluSphere(earth, radius, slices, stacks);
	    gl.glPopMatrix();
	}
	
	private void setColor(GL2 gl, float[] rgba) {
		gl.glMaterialfv(GL2.GL_FRONT, GL2.GL_AMBIENT, rgba, 0);
        gl.glMaterialfv(GL2.GL_FRONT, GL2.GL_SPECULAR, rgba, 0);
	}
	
	private void drawCube(GL2 gl, float x, float y, float z, float width, float height, float depth) {
		Vector3f n = new Vector3f();
		gl.glBegin(GL2.GL_QUADS);
			
		
		//top square
			n = calculateNormal(new Vector3f(x-width, y, z), new Vector3f(x, y, z), new Vector3f(x-width, y, z-depth));
			gl.glTexCoord2f(0.0f, 1.0f);
			gl.glNormal3f(-n.x, -n.y, -n.z);
			gl.glVertex3f( x, y, z ); // Top Right Of The Quad
			gl.glTexCoord2f(0.0f, 0.0f);
			gl.glNormal3f(-n.x, -n.y, -n.z);
			gl.glVertex3f( x-width, y, z ); // Top Left Of The Quad
			gl.glTexCoord2f(1.0f, 0.0f);
			gl.glNormal3f(-n.x, -n.y, -n.z);
			gl.glVertex3f( x-width, y, z-depth ); // Bottom Left Of The Quad
			gl.glTexCoord2f(1.0f, 1.0f);
			gl.glNormal3f(-n.x, -n.y, -n.z);
			gl.glVertex3f( x, y, z-depth ); // Bottom Right Of The Quad 
				
		
			//back square
			n = calculateNormal(new Vector3f(x, y-height, z-depth), new Vector3f( x-width, y-height, z-depth ), new Vector3f(x-width, y, z-depth));
			gl.glTexCoord2f(0.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y-height, z-depth ); // Bottom Left Of The Quad
			gl.glTexCoord2f(0.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y-height, z-depth ); // Bottom Right Of The Quad
			gl.glTexCoord2f(1.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y, z-depth ); // Top Right Of The Quad (Back)
			gl.glTexCoord2f(1.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y, z-depth ); // Top Left Of The Quad (Back)
			
		
			//left square
			n = calculateNormal(new Vector3f( x-width, y, z), new Vector3f( x-width, y, z-depth), new Vector3f( x-width, y-height, z-depth));
			gl.glTexCoord2f(0.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y, z ); // Top Right Of The Quad (Left)
			gl.glTexCoord2f(0.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y, z-depth ); // Top Left Of The Quad (Left)
			gl.glTexCoord2f(1.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y-height, z-depth ); // Bottom Left Of The Quad
			gl.glTexCoord2f(1.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y-height, z ); // Bottom Right Of The Quad 
			
			
		
			//right square
			n = calculateNormal(new Vector3f(x, y, z-depth), new Vector3f( x, y, z ), new Vector3f( x, y-height, z ));
			gl.glTexCoord2f(0.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y, z-depth ); // Top Right Of The Quad (Right)
			gl.glTexCoord2f(0.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y, z ); // Top Left Of The Quad
			gl.glTexCoord2f(1.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y-height, z ); // Bottom Left Of The Quad
			gl.glTexCoord2f(1.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y-height, z-depth ); // Bottom Right Of The Quad
			
			
			//bottom square
			n = calculateNormal(new Vector3f(x-width, y-height, z), new Vector3f(x, y-height, z), new Vector3f(x-width, y-height, z-depth));
			gl.glTexCoord2f(0.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y-height, z ); // Top Right Of The Quad
			gl.glTexCoord2f(0.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y-height, z ); // Top Left Of The Quad
			gl.glTexCoord2f(1.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x-width, y-height, z-depth ); // Bottom Left Of The Quad
			gl.glTexCoord2f(1.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f( x, y-height, z-depth ); // Bottom Right Of The Quad 
			
			
			//front square
			n = calculateNormal(new Vector3f(x-width, y-height, z), new Vector3f(x, y-height, z), new Vector3f(x, y, z));
			gl.glTexCoord2f(0.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f(x-width, y-height, z);
			gl.glTexCoord2f(0.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f(x, y-height, z);
			gl.glTexCoord2f(1.0f, 0.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f(x, y, z);
			gl.glTexCoord2f(1.0f, 1.0f);
			gl.glNormal3f(n.x, n.y, n.z);
			gl.glVertex3f(x-width, y, z);
				
		gl.glEnd();
	}
	
	private void setLighting(GL2 gl) {
		// Draw sphere (possible styles: FILL, LINE, POINT).
		// Prepare light parameters.
        float SHINE_ALL_DIRECTIONS = 1;
        float[] lightPos = {0, LIGHT_POSITION, 0, SHINE_ALL_DIRECTIONS};
        float[] lightColorAmbient = {0.2f, 0.2f, 0.2f, 1f};
        float[] lightColorSpecular = {0.8f, 0.8f, 0.8f, 1f};

        // Set light parameters.
        gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_POSITION, lightPos, 0);
        gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_AMBIENT, lightColorAmbient, 0);
        gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_SPECULAR, lightColorSpecular, 0);

        // Enable lighting in GL.
        gl.glEnable(GL2.GL_LIGHT1);
        gl.glEnable(GL2.GL_LIGHTING);

        // Set material properties.
        //setColor(gl, new float[] { 0.137255f, 0.556863f, 0.137255f});
        setColor(gl, new float[] {0.7f, 0.7f, 0.7f});
        gl.glMaterialf(GL2.GL_FRONT, GL2.GL_SHININESS, 0.5f);
	}
	
	private static Vector3f calculateNormal(Vector3f v0, Vector3f v1, Vector3f v2) {
	    Vector3f u = new Vector3f();
	    u.sub(v2, v0);
	    Vector3f w = new Vector3f();
	    w.sub(v1, v0);

	    Vector3f n = new Vector3f();
	    n.cross(u, w);
	    n.normalize(n);

	    if (Float.isNaN(n.x) || Float.isNaN(n.y) || Float.isNaN(n.z)) {
	        return new Vector3f(0,1,0);
	    }

	    return n;
	}
	
	private static void buildTempWorld() {
		//set up the physics
		BroadphaseInterface broadphase = new DbvtBroadphase();
		DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
		
		tempWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		buildGround(tempWorld);
		// set the gravity of our world
		tempWorld.setGravity(new Vector3f(0, -10, 0));
		
		// setup our collision shapes
		
		CollisionShape fallShape = new SphereShape(1.125f);
		CollisionShape longWall = new BoxShape(new Vector3f(128f, 10f, 4f));
		CollisionShape shortWall = new BoxShape(new Vector3f(4f, 10f, 68f));
		
		
		DefaultMotionState longWallLeft = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 5, -34), 1.0f))); 
		
		RigidBodyConstructionInfo longWallLeftRigidBodyCI = new RigidBodyConstructionInfo(0, longWallLeft, longWall, new Vector3f(0,0,0)); 
		longWallLeftRigidBodyCI.restitution = 0.25f;
		RigidBody longWallLeftRigidBody = new RigidBody(longWallLeftRigidBodyCI);  
		longWallLeftRigidBody.setFriction(1);
		
		tempWorld.addRigidBody(longWallLeftRigidBody); // add our left long wall to the dynamic world.. 
		
		DefaultMotionState longWallRight = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 5, 34), 1.0f))); 
		
		RigidBodyConstructionInfo longWallRightRigidBodyCI = new RigidBodyConstructionInfo(0, longWallRight, longWall, new Vector3f(0,0,0)); 
		longWallRightRigidBodyCI.restitution = 0.25f;
		RigidBody longWallRightRigidBody = new RigidBody(longWallRightRigidBodyCI); 
		longWallRightRigidBody.setFriction(1);
		
		tempWorld.addRigidBody(longWallRightRigidBody); // add our right long wall to the dynamic world.. 
		
		DefaultMotionState shortWallLeft = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(-64f, 5, 0), 1.0f))); 
		
		RigidBodyConstructionInfo shortWallLeftRigidBodyCI = new RigidBodyConstructionInfo(0, shortWallLeft, shortWall, new Vector3f(0,0,0)); 
		shortWallLeftRigidBodyCI.restitution = 0.25f;
		RigidBody shortWallLeftRigidBody = new RigidBody(shortWallLeftRigidBodyCI); 
		shortWallLeftRigidBody.setFriction(1);
		
		tempWorld.addRigidBody(shortWallLeftRigidBody); // add our Left short wall to the dynamic world..
		
		DefaultMotionState shortWallRight = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(64f, 5, 0), 1.0f))); 
		
		RigidBodyConstructionInfo shortWallRightRigidBodyCI = new RigidBodyConstructionInfo(0, shortWallRight, shortWall, new Vector3f(0,0,0)); 
		shortWallRightRigidBodyCI.restitution = 0.25f;
		shortWallRightRigidBodyCI.angularDamping = 0.95f;
		RigidBody shortWallRightRigidBody = new RigidBody(shortWallRightRigidBodyCI); 
		shortWallRightRigidBody.setFriction(1);
		
		tempWorld.addRigidBody(shortWallRightRigidBody); // add our right short wall to the dynamic world.. 

	    Transform trans = new Transform();
	    Renderer.cueBall.getMotionState().getWorldTransform(trans);
		
		// setup the motion state for the ball
		DefaultMotionState fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), trans.origin, 1.0f)));
		
		//This we're going to give mass so it responds to gravity 
		float mass = 0.1559f;
		
		Vector3f fallInertia = new Vector3f(0,0,0); 
		fallShape.calculateLocalInertia(mass,fallInertia); 
		
		RigidBodyConstructionInfo fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,fallShape,fallInertia); 
		fallRigidBodyCI.restitution = .5f;
		fallRigidBodyCI.angularDamping = .95f;
		RigidBody tcueBall = new RigidBody(fallRigidBodyCI);   
		tcueBall.setFriction(.5f); 
		RigidBody[] balls = new RigidBody[15];
		tcueBall.setUserPointer(tcueBall);

		//now we add it to our physics simulation 
		tempWorld.addRigidBody(tcueBall);
		
		//add rest of the balls
		// setup the motion state for the ball
		for(int i = 0; i < 15; i ++) {
			    Renderer.balls[i].getMotionState().getWorldTransform(trans);
				fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), trans.origin, 1.0f)));
				
				
				fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,fallShape,fallInertia); 
				fallRigidBodyCI.restitution = .5f;
				fallRigidBodyCI.angularDamping = .95f;
				balls[i] = new RigidBody(fallRigidBodyCI); 
				balls[i].setFriction(.5f); 

				//now we add it to our physics simulation 
				tempWorld.addRigidBody(balls[i]);
		}
		int count = 0;
		
		double distance = (Renderer.restPos.distance(Renderer.cuePos)/Renderer.marker_separation_constant) * MARKER_SPACING;
		double time = (double)1/6d;
		double force = (distance / time) * CUE_MASS;
		double angle = Math.atan2(Renderer.cuePos.y - Renderer.restPos.y, Renderer.cuePos.x - Renderer.restPos.x) + Math.toRadians(rotation-90f);
		float force_x = (float)(force * Math.cos(angle));
		float force_z = (float)(force * Math.sin(angle));
		tcueBall.activate();
		tcueBall.applyForce(new Vector3f(force_x, 0, force_z), new Vector3f(0,0,0));
		
		while(true) {

			tempWorld.stepSimulation(1/60.f, 10);
			tcueBall.getMotionState().getWorldTransform(trans);
			shot_path.add(new Vector3f(trans.origin));
			count++;
			Transform tempTrans = new Transform();
			for(int i = 0; i < 15; i++) {
				balls[i].getMotionState().getWorldTransform(trans);
				Renderer.balls[i].getMotionState().getWorldTransform(tempTrans);
				if(haveMoved[i] || trans != tempTrans) {
					if(!haveMoved[i]) {
						haveMoved[i] = true;
					}
					ball_shot_path.get(i).add(new Vector3f(trans.origin));
				}
			}
			if(count == 300) {
				break;
			}
		}
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
		RigidBody[] ground = new RigidBody[5];
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
	
	private void endGame(boolean eightBallLoss) {
		//clear frame
		JFrame winnerFrame = new JFrame( "VirtuPool" );
		winnerFrame.getContentPane().removeAll();
		
		JPanel menuPanel = new JPanel();
		menuPanel.setLayout(new GridLayout(1, 4));
		menuPanel.setBorder(new EmptyBorder(100,100,100,100));
		menuPanel.add(new JPanel());
		
		JButton newGameButton = new JButton("New Game");
		newGameButton.setPreferredSize(new Dimension(40, 40));
		newGameButton.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				 Renderer.cuePos = null;
				 Renderer.cueMoving = false;
				 Renderer.cuePos = null;
				 Renderer.restPos = null;
				 Renderer.stroke = false;
				 Renderer.stroke_prepped = true;
				 Renderer.stroke_counter = 0;
				 Renderer.balls = new RigidBody[15];
				 Renderer.ground = new RigidBody[5];
				 Renderer.initializePhysics();
				 winnerFrame.dispatchEvent(new WindowEvent(winnerFrame, WindowEvent.WINDOW_CLOSING));
			}
			
		});
		JPanel buttonPanel = new JPanel();
		buttonPanel.setLayout(new GridLayout(1, 3));
		buttonPanel.setBorder(new EmptyBorder(50,100,50,100));

		buttonPanel.add(new JPanel());
		buttonPanel.add(newGameButton);
		buttonPanel.add(new JPanel());
		menuPanel.add(buttonPanel);
		
		JButton quitButton = new JButton("Quit");
		quitButton.setPreferredSize(new Dimension(40, 40));
		quitButton.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				winnerFrame.dispatchEvent(new WindowEvent(winnerFrame, WindowEvent.WINDOW_CLOSING));	
				Renderer.frame.dispatchEvent(new WindowEvent(Renderer.frame, WindowEvent.WINDOW_CLOSING));	
			}
			
		});
		buttonPanel = new JPanel();
		buttonPanel.setLayout(new GridLayout(1, 3));
		buttonPanel.setBorder(new EmptyBorder(50,100,50,100));

		buttonPanel.add(new JPanel());
		buttonPanel.add(quitButton);
		buttonPanel.add(new JPanel());
		menuPanel.add(buttonPanel);
		
		JPanel messagePanel = new JPanel();
		messagePanel.setLayout(new BorderLayout());
		JLabel winnermessage = new JLabel("", JLabel.CENTER);
		winnermessage.setFont(new Font("Verdana",1,48));
		
		if(Renderer.p1turn) {
			if(eightBallLoss) {
				winnermessage.setText("Player 2 wins!");
			}
			winnermessage.setText("Player 1 wins!");
		}
		else {
			if(eightBallLoss) {
				winnermessage.setText("Player 1 wins!");
			}
			winnermessage.setText("Player 2 wins!");
		}
		winnerFrame.getContentPane().add(winnermessage, BorderLayout.CENTER);
		winnerFrame.getContentPane().add( menuPanel, BorderLayout.SOUTH );
		
		winnerFrame.setLocationRelativeTo(null);
		winnerFrame.setExtendedState(JFrame.MAXIMIZED_BOTH); 
		winnerFrame.setUndecorated(true);
		winnerFrame.setVisible( true );
	}
}
