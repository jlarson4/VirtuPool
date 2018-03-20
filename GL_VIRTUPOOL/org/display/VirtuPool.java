package display;

import graphics.Renderer;

import org.opencv.core.Core;

public class VirtuPool {
	//private VideoCapture capture = new VideoCapture();
	
	public static void main (String[] args) {

		// load the native OpenCV library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		Renderer.init();	
	}
}
