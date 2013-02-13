#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    
    bRoiChanging = false ; 
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// start from the front
	bDrawPointCloud = false;
    
    ofxUI_init() ;

    //loadCalibrationXml( ) ;
    
    roiArea = ofRectangle( 10 , 10 , 150 , 150 ) ; 
    bFullscreen = false ;
    
    fbo.allocate( ofGetWidth() , ofGetHeight() , GL_RGBA ) ;
    
    quadMaskFbo.allocate( roiArea.width , roiArea.height , GL_RGBA ) ;
    quadMaskFbo.begin() ;
        ofClear( 1 , 1 , 1 , 0 ) ;
    quadMaskFbo.end() ;
    
    loadQuadCalibration( ) ;
    quadMask.setup( "shader/composite_rgb" , roiArea ) ; 

}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew() && bRoiChanging == false ) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        grayImage.setROI( roiArea ) ;
		//grayImage.setROI( roiArea ) ;
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
            grayThreshNear.setROI( roiArea ) ;
			grayThreshFar = grayImage;
            grayThreshFar.setROI( roiArea ) ;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
            //grayImage.setROI( roiArea ) ;
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, minBlobSize, maxBlobSize, maxBlobs , bFindContourHoles );
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
    ofBackground( 0 , 0 , 0 ) ;
    if ( bFullscreen == false )
    {
        //Still calibrating
        ofSetColor(255, 255, 255);
        
        ofPushMatrix() ;
            ofSetColor( 255 , 255 ,255 ) ;
            grayImage.draw( 0 , 0 , grayImage.width , grayImage.height );
            //kinect.drawDepth( drawOffsetX , drawOffsetY , kinect.getWidth() , kinect.getHeight()  );
            ofEnableAlphaBlending() ;
            ofSetColor( 255 , 255 ,255 , 125 ) ;
            
            kinect.draw(0 , 0 , grayImage.width , grayImage.height );
            contourFinder.draw( roiArea.x , roiArea.y , kinect.width , kinect.height );
            ofSetColor( 255 , 255 , 0 ) ;
            ofNoFill ( ) ;
            ofRect( roiArea ) ;
            ofFill() ;
        ofPopMatrix() ;
        
        ofEnableSmoothing() ; 
        ofSetLineWidth( 2 ) ;
        
        ofPath quadArea ;
       
         if ( quadPoints.size() > 0 )
             quadArea.moveTo( quadPoints[0] ) ;
        
        for ( int i = 0 ; i < quadPoints.size() ; i++ )
        {
            ofSetColor( 125 ) ;
            ofCircle( quadPoints[ i ] , 4 ) ;
            quadArea.lineTo( quadPoints[i] ) ; 
            if ( i > 0 )
            {
                ofLine ( quadPoints[ i ] , quadPoints [ i - 1 ] ) ;
            }
        }
        
        if ( quadPoints.size() > 0 )
        {
            ofLine ( quadPoints[ 0 ] , quadPoints[ ( quadPoints.size() - 1 ) ] ) ;
            quadArea.lineTo( quadPoints[ 0 ] ) ;
        }
        
        quadArea.setFillColor( ofColor( 255 ) )  ;
        quadArea.setFilled( true ) ;
        quadArea.close( ) ;
        
        
        quadMaskFbo.begin() ;
            ofClear( 1 , 1 , 1 , 0 ) ;
            quadArea.draw( -roiArea.x , -roiArea.y ) ;
        quadMaskFbo.end() ;
        
        ofSetColor( 255 , 100 ) ;
        quadMaskFbo.draw( roiArea.x , roiArea.y ) ;
        

        // draw instructions
        ofSetColor(255, 255, 255);
        ofSetLineWidth( 1 ) ;
        stringstream reportStream;
        reportStream << " 'g' to toggle UI " << endl << " 'f' to toggle Fullscreen" << endl ;
        ofDrawBitmapString(reportStream.str(),ofGetWidth() - 200 , ofGetHeight() - 100 );
        
       // quadMask.drawMask( grayImage.getTextureReference() , quadMaskFbo.getTextureReference() , 0 , 0 , 255 ) ;
//        void ofxSimpleMask::drawMask ( ofTexture contentTex , ofTexture maskTex , float xOffset , float yOffset , float contentAlpha )
    }
    
    
    else
    {
        
        fbo.begin() ;
        
        ofSetColor( 0 , 0 , 0, fboFade ) ;
        ofRect( 0 , 0 , ofGetWidth() , ofGetHeight() ) ; 
        float _scale = kinect.getWidth() / roiArea.width ; 
        //Trying to overlap our objects
        ofPushMatrix( ) ;
            //ofScale( _scale , _scale , 1 ) ;
            //contourFinder.draw(  0 , 0 , kinect.width * _scale , kinect.width * _scaleddfd );
            // or, instead we can draw each blob individually,
            // this is how to get access to them
            int w = 640 ;
            int h = 480 ;
            ofColor col = ofColor::fromHsb( (int)( ofGetElapsedTimef() * 12.0f ) % 255 , 255 , 255 ) ;
        
            for (int i = 0; i < contourFinder.nBlobs; i++)
            {
                int nPts = contourFinder.blobs[i].pts.size() ;
                ofPath projectedShape ;
                for ( int p = 1 ; p < nPts ; p++ )
                {
                                       ofPoint p1 = contourFinder.blobs[i].pts[p-1] ;
                    p1.x /= roiArea.width ;
                    p1.y /= roiArea.height ;
                    p1.x *= ofGetWidth() ;
                    p1.y *= ofGetHeight() ;
                    
                    ofPoint p2 = contourFinder.blobs[i].pts[p] ;
                    p2.x /= roiArea.width ;
                    p2.y /= roiArea.height ;
                    p2.x *= ofGetWidth() ;
                    p2.y *= ofGetHeight() ;
                    
                    projectedShape.lineTo( p1 ) ;
                    projectedShape.lineTo( p2 ) ; 
                }
                //contourFinder.blobs[i].draw( 0 , 0 );
                projectedShape.close() ;
                projectedShape.setFilled( true ) ;
                projectedShape.setFillColor( col ) ;
                
                projectedShape.draw( ) ;

            }
        
        
        float areaRatio = (float) roiArea.height  / (float) roiArea.width ;
            
        
        for (int i = 0; i < contourFinder.nBlobs; i++)
        {
            for ( int k = 0 ; k < numOutlineLayers ; k++ )
            {
                float _outlineScale = outlineScale + ( (float)k * percPerLayer ) ;
                float xOffset = ofGetWidth() * (( _outlineScale-1.0f)/-2.0f );
                float yOffset = ofGetHeight() * (( _outlineScale-1.0f)/-2.0f ) * areaRatio ;
                int nPts = contourFinder.blobs[i].pts.size() ;
                ofPolyline projectedOutline ;
                
                float nWidth = contourFinder.blobs[i].boundingRect.width / roiArea.width ; 
                nWidth *= ofGetHeight() ;
                
                for ( int p = 1 ; p < nPts ; p++ )
                {
                    ofPoint p1 = contourFinder.blobs[i].pts[p-1] ;
                    p1.x /= roiArea.width ;
                    p1.y /= roiArea.height ;
                    p1.x *= ofGetWidth() * _outlineScale ;
                    p1.y *= ofGetHeight() * _outlineScale ;
                    p1.x += xOffset ;
                    p1.y += yOffset ;
                    
                    ofPoint p2 = contourFinder.blobs[i].pts[p] ;
                    p2.x /= roiArea.width ;
                    p2.y /= roiArea.height ;
                    p2.x *= ofGetWidth() * _outlineScale ;
                    p2.y *= ofGetHeight() * _outlineScale  ;
                    p2.x += xOffset ;
                    p2.y += yOffset ;
                    
                    projectedOutline.addVertex( p1 ) ;
                    projectedOutline.addVertex( p2 ) ; 
                }
                //contourFinder.blobs[i].draw( 0 , 0 );
                ofColor col2 = ofColor::fromHsb( (int)( ofGetElapsedTimef() * 12.0f * i ) % 255 , 255 , 255 ) ;

//                projectedShape.setFilled( false ) ;
  //              projectedShape.setFillColor( col ) ;
                ofSetColor( col2 ) ;
                projectedOutline.setClosed( true ) ; 
                projectedOutline.getSmoothed( outlineSmoothing ).draw( ) ;
            }
            
        }

            
        
        ofPopMatrix() ;
        fbo.end() ;
        
        ofSetColor( 255 , 255 , 255 ) ;
        fbo.draw( 0 , 0 ) ;
    }
}

void testApp::loadQuadCalibration ( )
{
    ofxXmlSettings quadXml ;
    quadXml.clear( ) ;
    bool bResult = quadXml.loadFile( "quadCalibration.xml") ;
    if ( bResult == false ) return ;
    
    int numQuadPoints = quadXml.getNumTags( "quadPointX" ) ;
    for ( int i = 0 ; i < numQuadPoints ; i++ )
    {
        float _x = quadXml.getValue( "quadPointX", -1 , i ) ;
        float _y = quadXml.getValue( "quadPointY", -1 , i ) ;
        quadPoints.push_back( ofPoint ( _x , _y ) ) ; 
    }
    
    //Fake a mouse released event
    mouseReleased( quadPoints[0].x + 5 , quadPoints[0].y + 5, 0 ) ;
    
    calculateRectangleFromQuadPoints( ) ; 
}

void testApp::drawPointCloud()
{
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
    
    
  }

void testApp::loadCalibrationXml()
{
    calibrateXml.loadFile( "calibration.xml" ) ;
    roiArea.x = calibrateXml.getValue( "x" , 0.0f ) ;
    roiArea.y = calibrateXml.getValue( "y" , 0.0f ) ;
    roiArea.width = calibrateXml.getValue( "width" , 0.0f ) ;
    roiArea.height = calibrateXml.getValue( "height" , 0.0f ) ;
}

void testApp::saveCalibrationXml()
{
    calibrateXml.clear() ;
    calibrateXml.setValue( "x" , roiArea.x ) ;
    calibrateXml.setValue( "y" , roiArea.y ) ;
    calibrateXml.setValue( "width" , roiArea.width ) ;
    calibrateXml.setValue( "height" , roiArea.height ) ;
    
    calibrateXml.saveFile( "calibration.xml" ) ;
}

//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

void testApp::ofxUI_init()
{
    float dim = 24;
	float xInit = OFX_UI_GLOBAL_WIDGET_SPACING;
    float length = 320-xInit;
    
    gui = new ofxUICanvas(0, 0, length+xInit, ofGetHeight());
	
    gui->addWidgetDown(new ofxUILabel("KINECT SLIDERS", OFX_UI_FONT_MEDIUM));
    gui->addFPSSlider("FPS SLIDER", length-xInit, dim*.25, 1000);
    gui->addWidgetDown(new ofxUIRangeSlider(length-xInit,dim, 0.0, 255.0, nearThreshold, farThreshold, "DEPTH RANGE"));
    gui->addSlider("KINECT MOTOR ANGLE", -30 , 30 , angle, length-xInit,dim);
    gui->addSlider("MAX NUM BLOBS", 0.0, 25, maxBlobs, length-xInit,dim);
    gui->addWidgetDown(new ofxUIRangeSlider(length-xInit,dim,10, kinect.getWidth() * kinect.getHeight() , minBlobSize , maxBlobSize  , "BLOB SIZE"));
    gui->addToggle( "OPENCV THRESHOLD", bThreshWithOpenCV , dim, dim);
    gui->addToggle( "DRAW POINT CLOUD", bDrawPointCloud, dim, dim);
    gui->addToggle( "FIND CONTOUR HOLES", bFindContourHoles, dim, dim);
    
    gui->addSlider("FBO FADE", 0.0, 255.0f , fboFade , length-xInit,dim);
    gui->addSlider("OUTLINE SCALE", 0.5, 3.0f , outlineScale , length-xInit,dim);
    gui->addSlider("OUTLINE SMOOTHING", 1.0, 100.0f , outlineSmoothing , length-xInit,dim);
    gui->addSlider("PERCENT SCALE PER LAYER", 0.0f, 1.0f , percPerLayer , length-xInit,dim);
    gui->addSlider("NUM LAYERS", 0.0f, 35.0f , numOutlineLayers , length-xInit,dim);
  
    ofAddListener( gui->newGUIEvent,this,&testApp::guiEvent );
    gui->loadSettings( "GUI/kinectSettings.xml" ) ;
}

//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.widget->getName();
	int kind = e.widget->getKind();

	//Sliders
	if(name == "KINECT MOTOR ANGLE" )
    {
        angle =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
        kinect.setCameraTiltAngle( angle ) ;
    }
    if(name == "MAX NUM BLOBS" ) maxBlobs =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
    if(name == "FBO FADE" ) fboFade =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
    if(name == "OUTLINE SCALE" ) outlineScale =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
    if(name == "OUTLINE SMOOTHING" ) outlineSmoothing =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
    if(name == "PERCENT SCALE PER LAYER" ) percPerLayer =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
    if(name == "NUM LAYERS" ) numOutlineLayers =  ( (ofxUISlider *) e.widget )->getScaledValue() ;
    
    //Toggles
    if(name == "OPENCV THRESHOLD" ) bThreshWithOpenCV = ( (ofxUIToggle *) e.widget )->getValue() ;
	if(name == "DRAW POINT CLOUD" ) bDrawPointCloud = ( (ofxUIToggle *) e.widget )->getValue() ;
    if(name == "FIND CONTOUR HOLES" ) bFindContourHoles = ( (ofxUIToggle *) e.widget )->getValue() ;
    
    //Range Sliders
    if(name == "DEPTH RANGE" )
    {
        farThreshold = ( (ofxUIRangeSlider *) e.widget )->getScaledValueLow() ;
        nearThreshold = ( (ofxUIRangeSlider *) e.widget )->getScaledValueHigh() ;
    }
    
    if(name == "BLOB SIZE" )
    {
        minBlobSize = ( (ofxUIRangeSlider *) e.widget )->getScaledValueLow() ;
        maxBlobSize = ( (ofxUIRangeSlider *) e.widget )->getScaledValueHigh() ;
    }

    gui->saveSettings( "GUI/kinectSettings.xml" ) ; 

}


//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    
	switch (key)
    {
		case 'g':
        case 'G':
            gui->toggleVisible() ;
            break ;
            
        case 'f':
        case 'F':
            bFullscreen = !bFullscreen ;
            ofToggleFullscreen() ;
            break ;
            
        case 'r':
        case 'R':
            quadPoints.clear() ;
            break ; 
            
	}
}



//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
    if ( gui->isVisible() == true ) return ; 
    bRoiChanging = true ;
    
    ofPoint m = ofPoint ( x , y ) ;
    checkMouseAgainstQuadPoints( m ) ; 
    
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    if ( gui->isVisible() == true ) return ;
    
    bRoiChanging = true ; 
    ofPoint m = ofPoint ( x , y ) ;
    
    //We only want to have a total of 4 points
    if ( quadPoints.size() < 4)
    {
        //add the point as a new one
        quadPoints.push_back( m ) ;
    }
    else
    {
        checkMouseAgainstQuadPoints( m ) ; 
    }
    
    
    //roiArea = ofRectangle( x - drawOffsetX , y - drawOffsetY  , 0 , 0 ) ;
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{
    if ( gui->isVisible() == true ) return ;
    //roiArea.width = x - roiArea.x - drawOffsetX ;
    //roiArea.height = y - roiArea.y - drawOffsetY ;
    
    checkMouseAgainstQuadPoints(ofPoint ( x , y ) ) ; 
    //If all points are in place
    if ( quadPoints.size() == 4 )
    {
        calculateRectangleFromQuadPoints( ) ;
        //saveCalibrationXml() ;
        saveQuadCalibration( ) ;
        bRoiChanging = false ;
        
        roiAreaFbo.allocate( roiArea.width , roiArea.height , GL_RGBA ) ;
        roiAreaFbo.begin( ) ;
            ofClear( 1 , 1 , 1, 0 ) ;
        roiAreaFbo.end( ) ;
    }
     
}

void testApp::calculateRectangleFromQuadPoints( )
{
    if ( quadPoints.size() == 4 )
    {
        ofRectangle quadPointsRect = ofRectangle ( 100000 , 100000 , -1000000 , -1000000 ) ;
        for ( int i = 0 ; i < quadPoints.size() ; i++ )
        {
            if ( quadPoints[i].x < quadPointsRect.x )
                quadPointsRect.x = quadPoints[i].x ;
            if ( quadPoints[i].y < quadPointsRect.y )
                quadPointsRect.y = quadPoints[i].y ;
            if ( quadPoints[i].x > quadPointsRect.width )
                quadPointsRect.width = quadPoints[i].x ;
            if ( quadPoints[i].y > quadPointsRect.height )
                quadPointsRect.height = quadPoints[i].y ;
        }
        
        roiArea = ofRectangle ( quadPointsRect.x , quadPointsRect.y , quadPointsRect.width - quadPointsRect.x , quadPointsRect.height - quadPointsRect.y ) ;
        
        quadMaskFbo.allocate( roiArea.width , roiArea.height , GL_RGBA ) ;
        quadMaskFbo.begin() ;
        ofClear( 1 , 1 , 1 , 0 ) ;
        quadMaskFbo.end() ;
        cout << "roiArea : " << roiArea.x << " , " << roiArea.y << " , " << roiArea.width << " , " << roiArea.height << endl ;
    }
    else
    {
        cout << "not enough quad Points !! " << endl ; 
    }
}


void testApp::checkMouseAgainstQuadPoints ( ofPoint m )
{
    float closestDist = 100000.0f ;
    int closestIndex = 0 ;
    for ( int i = 0 ; i < quadPoints.size() ; i++ )
    {
        float dist = quadPoints[ i ].distance( m ) ;
        if ( dist < closestDist )
        {
            closestDist = dist ;
            closestIndex = i ;
        }
    }
    quadPoints[ closestIndex ] = m ;
}

void testApp::saveQuadCalibration( )
{
    ofxXmlSettings quadXml ;
    quadXml.clear( ) ;
    for ( int i = 0 ; i < quadPoints.size() ; i++ )
    {
        quadXml.setValue( "quadPointX", quadPoints[i].x , i ) ;
        quadXml.setValue( "quadPointY", quadPoints[i].y , i ) ;
    }
    quadXml.saveFile( "quadCalibration.xml" ) ; 
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}