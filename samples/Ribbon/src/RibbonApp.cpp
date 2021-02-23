#include <time.h>

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/gl/wrapper.h"
#include "cinder/CinderGlm.h"
#include "cinder/Color.h"
#include "cinder/Rand.h"
#include "cinder/GeomIo.h"
#include "cinder/ImageIo.h"
#include "cinder/Log.h"
#include "cinder/Utilities.h"
#include "cinder/System.h"
#include "cinder/TriMesh.h"
#include "cinder/Xml.h"
#include "cinder/Camera.h"
#include "cinder/Arcball.h"
#include "cinder/CameraUi.h"
#include "cinder/AxisAlignedBox.h"

#include "BSplinePatch.h"
#include "BSplineSurface.h"

using namespace ci;
using namespace ci::app;
using namespace glm;
using namespace std;

namespace cinder
{
	vec2 fromPolar( vec2 polar )
	{
		return vec2( math<float>::cos( polar.y ) *  polar.x , math<float>::sin( polar.y ) *  polar.x );
	}
}

class RibbonApp : public App {
public:
	RibbonApp() {}
	virtual ~RibbonApp() {}
	
	void setup();
	void mouseMove( MouseEvent event );
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );
	void keyDown( KeyEvent event );
	void resize();
	void update();
	void draw();
	
private:
	void drawWorldFrame();
	void updateSplinePatch();
	void generateKnots( std::vector<float>& knots, const size_t count );
	
	CameraOrtho				mStaticCam;
	CameraPersp				mCam;
	CameraUi				mMayaCam;
	ivec2					mMousePos;
	DisplayRef				mWindowDisplay;
	
	float					mFps;
	bool					mDrawBezierPatch;
	bool					mDrawWireframe;
	bool					mDrawParams;
	bool					mEnableBackfaceCulling;
	bool					mEnableAdditiveBlending;
	bool					mPause;
	bool					mLoopU;
	bool					mLoopV;
	bool					mOpenU;
	bool					mOpenV;
	float					mKnotU;
	float					mKnotV;
	float					mRadiusMin;
	float					mRadiusMax;
	float					mHullAmplitude;
	float					mAmplitudeX;
	float					mAmplitudeY;
	float					mAmplitudeZ;
	float					mFrequencyX;
	float					mFrequencyYMin;
	float					mFrequencyYMax;
	float					mFrequencyZ;
	float					mTimeScale;
	float					mTorsionXMin;
	float					mTorsionXMax;
	float					mTorsionXSpeed;
	float					mMeshScaleX;
	float					mMeshScaleY;
	float					mMeshScaleZ;
	float					mAlpha;
	float					mStaticCam_left;
	float					mStaticCam_right;
	float					mStaticCam_top;
	float					mStaticCam_bottom;
	vec3					mStaticCam_position;
	vec3					mStaticCam_direction;
	vec3					mLightPosition;
	Color					mAmbientColor;
	Color					mDiffuseColor;
	Color					mSpecularColor;
	ColorA					mClearColor;
	int32_t					mLaticeWidth;
	int32_t					mLaticeLength;
	int32_t					mMeshWidth;
	int32_t					mMeshLength;
	int32_t					mSplineDegreeU;
	int32_t					mSplineDegreeV;
	int32_t					mCameraIndex;
	params::InterfaceGlRef	mParams;

	bool					mIsReady = {false};
	BSplinePatch			mBSplineRect;
	BSplineSurface			mSurfaceMesh;
	std::vector<ci::vec3>	mCtrlPoints;
	std::vector<float>		mCtrlKnotsV;
	ci::gl::VboMeshRef		mMesh;
	ci::gl::GlslProgRef		mShader;
	ci::gl::BatchRef		mMeshBatch;
	
	float					mKnot0;
	float					mKnot1;
	float					mKnot2;
};

void RibbonApp::setup()
{
	getWindow()->getSignalResize().connect(std::bind(&RibbonApp::resize, this));
	getWindow()->getSignalDisplayChange().connect(std::bind(&RibbonApp::resize, this));
	mWindowDisplay = getWindow()->getDisplay();
	
	mFps = 0.0;
	mEnableBackfaceCulling = false;
	mEnableAdditiveBlending = true;
	mDrawBezierPatch = false;
	mDrawWireframe = false;
	mPause = false;
	mLaticeWidth = 8;
	mLaticeLength = 20;
	mDrawParams = true;
	mMeshWidth = 60;
	mMeshLength = 30;
	mSplineDegreeU = 3;
	mSplineDegreeV = 2;
	mLoopU = false;
	mLoopV = true;
	mOpenU = true;
	mOpenV = false;
	mRadiusMin = 150.0f;
	mRadiusMax = 50.0f;
	mFrequencyX = 12.0f;
	mFrequencyYMin = 35.0f;
	mFrequencyYMax = 45.0f;
	mFrequencyZ = 6.5f;
	mHullAmplitude = 120.0f;
	mAmplitudeX = 425.0f;
	mAmplitudeY = 20.0f;
	mAmplitudeZ = 52.0f;
	mMeshScaleX = 500.0f;
	mMeshScaleY = 45.0f;
	mMeshScaleZ = 85.0f;
	mTorsionXMin = -50.0f;
	mTorsionXMax = 125.0f;
	mTorsionXSpeed = 7.0f;
	mTimeScale = 500.0f;
	mAlpha = 0.6f;
	mAmbientColor = Color(30.0/255.0, 80.0/255.0, 90.0/255.0);
	mDiffuseColor = Color(56.0/255.0, 120.0/255.0, 110.0/255.0);
	mSpecularColor = Color(216.0/255.0, 252.0/255.0, 238.0/255.0);
	mClearColor = ColorA(8.0f/255.0, 26.0/255.0, 42.0/255.0, 1.0);
	
	mCameraIndex = 1;
	mStaticCam_left = -6.0f;
	mStaticCam_right = 16.0f;
	mStaticCam_top = 6.0f;
	mStaticCam_bottom = -8.0f;
	mLightPosition = vec3(1);
	
	mKnot0 = 0.13 * 100;
	mKnot1 = 0.45 * 100;
	mKnot2 = 0.66 * 100;
	
	try {
		mShader = gl::GlslProg::create(app::loadAsset("shaders/ribbon.vert"), app::loadAsset("shaders/ribbon.frag"));
	}
	catch ( gl::GlslProgCompileExc& exc ) {
		console() << "ribbon shader compile error: " << exc.what();
	}
	catch ( gl::GlslNullProgramExc& exc ) {
		console() << "ribbon shader is null: " << exc.what();
	}
	
	this->updateSplinePatch();
	this->generateKnots( mCtrlKnotsV, mLaticeLength - mSplineDegreeV );
	mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
	mSurfaceMesh = BSplineSurface( const_cast<BSplinePatch*>(&mBSplineRect), ivec2(mMeshWidth, mMeshLength) ).texCoords( vec2(0,0), vec2(1,1) );
	TriMeshRef surface = TriMesh::create( mSurfaceMesh );
	mMesh = gl::VboMesh::create( *surface.get() );
	mMeshBatch = gl::Batch::create( mMesh, mShader );
	
	vec3 center = surface->calcBoundingBox().getCenter();
	mCam.setEyePoint(vec3(center.x, 0.0f, 20.0f));
	mCam.lookAt(center);
	mCam.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
	mMayaCam.setCamera(&mCam);
	mStaticCam_position = vec3(center.x, 0.0f, 20.0f);
	mStaticCam_direction = vec3(0,0,-1.0f);
	
	mStaticCam.setEyePoint(mStaticCam_position);
	mStaticCam.setOrtho(mStaticCam_left, mStaticCam_right, mStaticCam_bottom, mStaticCam_top, 1.0, 1000.0f);
	mStaticCam.setWorldUp(vec3(0,1,0));
	mStaticCam.setViewDirection(mStaticCam_direction);
	
	mParams = params::InterfaceGl::create(this->getWindow(), "Parameters", ivec2(350, 600));
//	mParams.setOptions("", "");
	mParams->addParam("fps: ", &mFps);
	
	vector<string> cameras;
	cameras.push_back("Maya cam");
	cameras.push_back("Static cam");
	mParams->addParam("camera", cameras, &mCameraIndex);
	mParams->addParam("draw bezier patch latice", &mDrawBezierPatch);
	mParams->addParam("draw wireframe", &mDrawWireframe);
	mParams->addParam("enable backface culling", &mEnableBackfaceCulling);
	mParams->addParam("use additive blending", &mEnableAdditiveBlending);
	mParams->addParam("pause", &mPause);
	mParams->addParam("knot 0", &mKnot0);
	mParams->addParam("knot 1", &mKnot1);
	mParams->addParam("knot 2", &mKnot2);
	mParams->addParam("patch width", &mLaticeWidth);
	mParams->addParam("patch length", &mLaticeLength);
	mParams->addParam("mesh width", &mMeshWidth);
	mParams->addParam("mesh length", &mMeshLength);
	mParams->addParam("spline degree U", &mSplineDegreeU);
	mParams->addParam("spline degree V", &mSplineDegreeV);
	mParams->addParam("loop U", &mLoopU);
	mParams->addParam("loop V", &mLoopV);
	mParams->addParam("open U", &mOpenU);
	mParams->addParam("open V", &mOpenV);
	mParams->addParam("minimum radius", &mRadiusMin);
	mParams->addParam("maximum radius", &mRadiusMax);
	mParams->addParam("frequency X", reinterpret_cast<float*>(&this->mFrequencyX));
	mParams->addParam("frequency Y min", &mFrequencyYMin);
	mParams->addParam("frequency Y max", &mFrequencyYMax);
	mParams->addParam("frequency Z", &mFrequencyZ);
	mParams->addParam("hull amplitude", &mHullAmplitude);
	mParams->addParam("amplitude X", &mAmplitudeX);
	mParams->addParam("amplitude Y", &mAmplitudeY);
	mParams->addParam("amplitude Z", &mAmplitudeZ);
	mParams->addParam("mesh scale X", &mMeshScaleX);
	mParams->addParam("mesh scale Y", &mMeshScaleY);
	mParams->addParam("mesh scale Z", &mMeshScaleZ);
	mParams->addParam("min torsion on X", &mTorsionXMin);
	mParams->addParam("max torsion on X", &mTorsionXMax);
	mParams->addParam("torsion speed", &mTorsionXSpeed);
	mParams->addParam("time scale", &mTimeScale);
	mParams->addSeparator();
	mParams->addText("Shader parameters");
	mParams->addParam("alpha transparency", (float*)&mAlpha, "min=0.0 max=1.0");
	mParams->addParam("ambient color", &mAmbientColor);
	mParams->addParam("diffuse color", &mDiffuseColor);
	mParams->addParam("specular color", &mSpecularColor);
	mParams->addParam("clear color", &mClearColor);
	mParams->addSeparator();
	mParams->addText("Static cam parameters");
	mParams->addParam("matrix left", &mStaticCam_left);
	mParams->addParam("matrix right", &mStaticCam_right);
	mParams->addParam("matrix bottom", &mStaticCam_bottom);
	mParams->addParam("matrix top", &mStaticCam_top);
	mParams->addParam("camera position", &mStaticCam_position);
	mParams->addParam("camera direction", &mStaticCam_direction);
	
	mIsReady = true;
}

void RibbonApp::mouseMove( MouseEvent event )
{
	mMousePos = event.getPos();
	
	float x = static_cast<float>(mMousePos.x) / static_cast<float>(getWindowSize().x);
	float y = static_cast<float>(mMousePos.y) / static_cast<float>(getWindowSize().y);
	mLightPosition = vec3(0, y * 4.0f, 3.0 - x * 6.0f);
}

void RibbonApp::mouseDrag( MouseEvent event )
{
	if (0 == mCameraIndex) {
		mMousePos = event.getPos();
		mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
	}
}

void RibbonApp::mouseUp( MouseEvent event )
{
}

void RibbonApp::mouseDown( MouseEvent event )
{
	if (0 == mCameraIndex) {
		mMayaCam.mouseDown( event.getPos() );
	}
}

void RibbonApp::keyDown( KeyEvent event )
{
	if (event.isMetaDown() && event.getCode() == KeyEvent::KEY_w)
		this->quit();	// close window behavior
	
	switch(event.getCode()){
		case KeyEvent::KEY_p:
			mDrawParams = !mDrawParams;
			break;
			
		case KeyEvent::KEY_q:
			break;
			
		case KeyEvent::KEY_w:
			break;
			
		case KeyEvent::KEY_f:
			setFullScreen(!isFullScreen());
			break;
			
		case KeyEvent::KEY_c:
			{
				AxisAlignedBox aabb;
				mSurfaceMesh >> geom::Bounds(&aabb);
				vec3 center = aabb.getCenter();
				
				mCam.setPerspective( 60, getWindowAspectRatio(), 1, 1000 );
				mCam.setAspectRatio( getWindowAspectRatio() );
				mCam.setEyePoint(vec3(center.x, 0.0f, 20.0f));
				mCam.lookAt(center);
				mMayaCam.setCamera(&mCam);
			}
			break;
		
		default:
			break;
	}
}

void RibbonApp::resize()
{
	gl::setMatricesWindow( getWindowSize() );
	
	mCam.setPerspective( 60, getWindowAspectRatio(), 1, 1000 );
	mCam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCamera(&mCam);
	
	mStaticCam.setEyePoint(mStaticCam_position);
	mStaticCam.setViewDirection(mStaticCam_direction);
}

void RibbonApp::update()
{
	if (!mIsReady) return;
	
	if (getWindow()->getDisplay() != mWindowDisplay) {
		mWindowDisplay = getWindow()->getDisplay();
		// HACK: this is required since MacOS Catalina due to a change in how OSX reports the display content scaling, sadly
		ivec2 size = getWindowSize();
		setWindowSize(size.x, size.y-1);
		setWindowSize(size.x, size.y);
	}

	mFps = static_cast<int32_t>(math<float>::floor(app::App::get()->getAverageFps()));
	
	if (!mPause) {
		this->updateSplinePatch();
		this->generateKnots( mCtrlKnotsV, mLaticeLength - mSplineDegreeV );
	}
	
	//mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
	mBSplineRect.updateControlPoints( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength) );
	//mBSplineRect = BSplinePatch(mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mCtrlKnotsV);
	mSurfaceMesh = BSplineSurface( const_cast<BSplinePatch*>(&mBSplineRect), ivec2(mMeshWidth, mMeshLength) ).texCoords( vec2(0,0), vec2(1,1) );
	TriMeshRef surface = TriMesh::create( mSurfaceMesh );
	//TriMeshRef surface = TriMesh::create( geom::VertexNormalLines( surfaceMesh, 1.0 ) );
	mMesh = gl::VboMesh::create( *surface.get() );
	mMeshBatch = gl::Batch::create( mMesh, mShader );
	
	if (1 == mCameraIndex) {
		mStaticCam.setEyePoint(mStaticCam_position);
		mStaticCam.setOrtho(mStaticCam_left, mStaticCam_right, mStaticCam_bottom, mStaticCam_top, 1.0, 1000.0f);
		mStaticCam.setWorldUp(vec3(0,1,0));
		mStaticCam.setViewDirection(mStaticCam_direction);
	}
}

void RibbonApp::draw()
{
	if (!mIsReady) return;
	
	gl::clear(mClearColor, true);
	if (mEnableAdditiveBlending) {
		gl::enableAdditiveBlending();
	}
	else {
		gl::enableAlphaBlending();
	}
	gl::pushMatrices();
	if (0 == mCameraIndex) {
		gl::setMatrices(mMayaCam.getCamera());
	}
	else if (1 == mCameraIndex) {
		gl::setMatrices(mStaticCam);
	}
	
	if (mEnableBackfaceCulling) {
		gl::enableFaceCulling();
		gl::cullFace(GL_BACK);
	}
	
	if (mDrawWireframe) {
		gl::enableWireframe();
		gl::color(1,1,1,1);
		gl::draw( mMesh );
		gl::disableWireframe();
	}
	else if (mShader) {
		mShader->uniform("lightPosition", vec4(mLightPosition,1.0f));
		mShader->uniform("eyePosition", mMayaCam.getCamera().getEyePoint());
		mShader->uniform("alpha", mAlpha);
		mShader->uniform("ambientColor", mAmbientColor);
		mShader->uniform("diffuseColor", mDiffuseColor);
		mShader->uniform("specularColor", mSpecularColor);
		mMeshBatch->draw();
	}
	
	if (mEnableBackfaceCulling)
		gl::enableFaceCulling( false );
	
	if (mDrawBezierPatch) {
		gl::pointSize(5.0f);
		gl::begin(GL_POINTS);
		gl::color(0,1,1,1);
		uint32_t i,j;
		for(i = 0; i != mLaticeWidth; ++i) {
			for(j = 0; j != mLaticeLength; ++j) {
				gl::vertex(mCtrlPoints[i * mLaticeLength + j]);
			}
		}
		gl::end();
	}
	gl::popMatrices();

	if (mDrawParams) {
		mParams->draw();
	}
}

void RibbonApp::drawWorldFrame()
{
	float s = 5.0f;
	gl::color(1,0,0,1);
	gl::drawLine(vec3(0), vec3(s,0,0));
	gl::color(0,1,0,1);
	gl::drawLine(vec3(0), vec3(0,s,0));
	gl::color(0,0,1,1);
	gl::drawLine(vec3(0), vec3(0,0,s));
}

void RibbonApp::updateSplinePatch()
{
	float t = (float) getElapsedSeconds() * (mTimeScale * 0.01f);
	float ampX = mAmplitudeX * 0.01f;
	float ampY = mAmplitudeY * 0.01f;
	float ampZ = mAmplitudeZ * 0.01f;
	float hull_amp = mHullAmplitude * 0.01f;
	float freqX = mFrequencyX * 0.01f;
	float freqY_min = mFrequencyYMin * 0.01f;
	float freqY_max = mFrequencyYMax * 0.01f;
	float freqY = freqY_min;
	float freqZ = mFrequencyZ * 0.01f;
	float scale_x = mMeshScaleX * 0.01f;
	float scale_y = mMeshScaleY * 0.01f;
	float scale_z = mMeshScaleZ * 0.01f;
	float torsion_min_x = mTorsionXMin * 0.01f;
	float torsion_max_x = mTorsionXMax * 0.01f;
	float torsion_speed = mTorsionXSpeed * 0.01f;
	float radius_min = mRadiusMin * 0.01f;
	float radius_max = mRadiusMax * 0.01f;
	
	float weights[24] = {0.1, 0.115, 0.175, 0.33, 0.25, 0.275, 0.33, 0.39, 0.43, 0.5, 0.54, 0.42,
						0.5, 0.575, 0.6, 0.67, 0.6, 0.45, 0.415, 0.4, 0.375, 0.35, 0.15, 0.179};
	
	size_t meshSize = mLaticeWidth * mLaticeLength;
	if (mCtrlPoints.size() != meshSize)
		mCtrlPoints.resize( meshSize );
	
	if (mLoopV) {
		uint32_t i,j;
		float x, y, z;
		float r_x, radius, theta, t_x;
		float two_pi = static_cast<float>(2.0 * M_PI);
		t_x = two_pi / mLaticeLength;
		float one_pi = static_cast<float>(M_PI);
		r_x = 0.0f;
		float s = 0.5f + (math<float>::sin(t * torsion_speed) * 0.5f);
		float torsion_x = lerp(torsion_min_x, torsion_max_x, s);
		ampX *= (s + 0.25f);
		for(i = 0; i != mLaticeWidth; ++i) {
			theta = r_x;
			r_x += torsion_x;
			for(j = 0; j != mLaticeLength; ++j) {
				float aX = (0.75f * ampX) + (3.5f * weights[j]);
				float alpha = theta <= one_pi? (theta/one_pi): (two_pi - theta) / one_pi;
				radius = lerp(radius_min, radius_max, s) + lerp(1.0f, hull_amp + math<float>::sin(j * freqY), alpha);
				vec2 hull_pt = ci::fromPolar( vec2(radius, theta) );
				x = i * scale_x;
				y = hull_pt.y * scale_y;
				y += math<float>::cos((weights[j]) + i + t * freqX) * aX;
				y += math<float>::sin((t * freqZ) + (i * 0.75f));
				z = hull_pt.x * scale_z;
				mCtrlPoints[i * mLaticeLength + j] = vec3(x,y,z);
				
				theta += t_x;
			}
		}
	}
	else {
		uint32_t i,j;
		float fX,fY,aX, x,y,z;
		for(i = 0; i != mLaticeWidth; ++i) {
			for(j = 0; j != mLaticeLength; ++j) {
				fX = freqX;
				fY = j * t * math<float>::sin(0.15f * weights[j]);
				aX = (0.75f * ampX) + (3.5f * weights[j]);
				x = i * scale_x;
				y = scale_y * math<float>::cos((1.0f * weights[j]) + i + t * fX) * aX;
				z = (j * scale_z) * (1.0f + math<float>::cos(i + t * freqY) * ampY);		//! position point-z
				y += (1.0 * weights[j]) + math<float>::sin(fY * freqZ) * ampZ;				//! add torsion-x to y
				mCtrlPoints[i * mLaticeLength + j] = vec3(x,y,z);
			}
		}
	}
}

void RibbonApp::generateKnots( std::vector<float>& knots, const size_t count )
{
	knots.clear();
	knots.resize(count);
	float tmp = 0.0f;
	float delta = 1.1f / static_cast<float>(count);
	for (uint32_t i = 0; i < knots.size(); i++) {
		knots[i] = tmp + (i * delta);
		if (7 == i) {
			knots[i] = mKnot2 / 100;
		}
		else if (5 == i) {
			knots[i] = mKnot1 / 100;
		}
		else if (3 == i) {
			knots[i] = mKnot0 / 100;
		}
	}
}

CINDER_APP( RibbonApp, RendererGl(),
[&]( App::Settings *settings ) {
	settings->setDisplay(Display::getMainDisplay());
	settings->setHighDensityDisplayEnabled( false );
	settings->setWindowSize(1600, 1080);
	settings->setFrameRate(60);
	settings->setTitle("RibbonApp");
	
	// seed random number generators
	srand(time(NULL));
	Rand::randSeed(time(NULL));
})
