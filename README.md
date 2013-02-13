/*

ofxKinect_ProjectSurfaces
by Ben McChesney December 2012

based off of ofxKinect by Theo Watson + Other : github.com/ofxKinect

This is a tool to project over simple openCV object by calibrating and cropping the kinect space to the projector space.
Works only with a kinect. Only tested with kinect for xbox.
*/

Toggle the GUI off with 'g'
with the GUI off draw the square rectangle ( it will turn yellow ) that's the area of the projector screen
Toggling 'f' will bring the app into fullscreen mode and scale the yellow square's openCV blobs to fit the screen

Example using ofxKinect ( libfreenect way ) 
	Make sure there is a gray quad drawn on the screen with a yellow bounding box
	This bounding box will get scaled to the entire projector space when the app goes fullscreen
	Eventually the texture will be wrapped to a QUAD and not a rectangle for flexibility of mounting a projector.
	
Example using ofxOpenNI ( OpenNI and skeleton )

**Still very much in development