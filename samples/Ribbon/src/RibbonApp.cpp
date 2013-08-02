#include <time.h>
#include <boost/bind.hpp>

#include "cinder/app/AppNative.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Color.h"
#include "cinder/Rand.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/System.h"
#include "cinder/TriMesh.h"
#include "cinder/Xml.h"
#include "cinder/Camera.h"
#include "cinder/Arcball.h"
#include "cinder/MayaCamUI.h"
#include "cinder/AxisAlignedBox.h"

#include "BSplinePatch.h"
#include "SurfaceTriMesh.h"
#include "SurfaceVboMesh.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace cg;

namespace cinder { namespace gl {

void enableBackFaceCulling()
{
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
}

void enableFrontFaceCulling()
{
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
}

void disableCulling()
{
	glDisable(GL_CULL_FACE);
}
	
} }

class RibbonApp : public AppNative {
public:
	RibbonApp() {}
	virtual ~RibbonApp() {}
	
	void prepareSettings( AppBasic::Settings *settings );
	void setup();
	void mouseMove( MouseEvent event );
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );
	void resize();
    void keyDown( KeyEvent event );
	void update();
	void draw();
	
private:
	void drawWorldFrame();
	void updateSplinePatch();
	
	CameraOrtho				mStaticCam;
	CameraPersp				mCam;
	MayaCamUI				mMayaCam;
	Vec2i					mMousePos;
	
	float					mFps;
	bool						mDrawBezierPatch;
	bool						mDrawWireframe;
	bool						mDrawParams;
	bool						mEnableBackfaceCulling;
	bool						mEnableAdditiveBlending;
	bool						mPause;
	bool						mLoopU;
	bool						mLoopV;
	bool						mOpenU;
	bool						mOpenV;
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
	Vec3f					mStaticCam_position;
	Vec3f					mStaticCam_direction;
	Vec3f					mLightPosition;
	Color					mAmbientColor;
	Color					mDiffuseColor;
	Color					mSpecularColor;
	ColorA					mClearColor;
	int32_t					mSurfaceMeshType;
	int32_t					mLaticeWidth;
	int32_t					mLaticeLength;
	int32_t					mMeshWidth;
	int32_t					mMeshLength;
	int32_t					mSplineDegreeU;
	int32_t					mSplineDegreeV;
	int32_t					mCameraIndex;
//	int32_t					mHullEdges;
	params::InterfaceGlRef	mParams;

	bool mIsReady;
	cg::BSplinePatch bsplineRect;
	cg::SurfaceTriMesh surfaceMesh;
	cg::SurfaceVboMesh surfaceVbo;
	cg::ControlPointLatice mCtrlPoints;
	ci::gl::GlslProg mShader;
};

void RibbonApp::prepareSettings( Settings *settings )
{
	mIsReady = false;
	
	settings->setWindowSize(1600, 1080);
	settings->setFrameRate(60);
	settings->setTitle("RibbonApp");
	
	// seed random number generators
	srand(time(NULL));
	Rand::randSeed(time(NULL));
}

void RibbonApp::setup()
{
	// Add Mac resource paths...
	fs::path path;
	path = this->getAppPath() / ".." / ".." / ".." / ".." / "assets";
	this->addAssetDirectory(path);
	
	this->getWindow()->getSignalResize().connect(std::bind(&RibbonApp::resize, this));
	
	mFps = 0.0;
	mSurfaceMeshType = 1;
	mEnableBackfaceCulling = false;
	mEnableAdditiveBlending = true;
	mDrawBezierPatch = false;
	mDrawWireframe = false;
	mPause = false;
//	mHullEdges = 10;
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
	mTimeScale = 100.0f;
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
	mLightPosition = Vec3f::one();
	
	try {
		mShader = gl::GlslProg(app::loadAsset("shaders/ribbon.vert"), app::loadAsset("shaders/ribbon.frag"));
	}
	catch ( gl::GlslProgCompileExc& exc ) {
		console() << "ribbon shader compile error: " << exc.what();
	}
	catch ( gl::GlslNullProgramExc& exc ) {
		console() << "ribbon shader is null: " << exc.what();
	}
	
	this->updateSplinePatch();
	bsplineRect.create(mCtrlPoints, mLaticeWidth, mLaticeLength, mSplineDegreeU, mSplineDegreeV, mLoopU, mLoopV, mOpenU, mOpenV);
	surfaceMesh = SurfaceTriMesh(bsplineRect, mMeshWidth, mMeshLength, Vec2f(0,0), Vec2f(1,1));
	surfaceVbo = SurfaceVboMesh(bsplineRect, mMeshWidth, mMeshLength, Vec2f(0,0), Vec2f(1,1));
	
	Vec3f center = surfaceMesh.trimesh().calcBoundingBox().getCenter();
	mCam.setEyePoint(Vec3f(center.x, 0.0f, 20.0f));
	mCam.setCenterOfInterestPoint(center);
	mCam.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
	mMayaCam.setCurrentCam(mCam);
	mStaticCam_position = Vec3f(center.x, 0.0f, 20.0f);
	mStaticCam_direction = Vec3f(0,0,-1.0f);
	
	mStaticCam.setEyePoint(mStaticCam_position);
	mStaticCam.setOrtho(mStaticCam_left, mStaticCam_right, mStaticCam_bottom, mStaticCam_top, 1.0, 1000.0f);
	mStaticCam.setWorldUp(Vec3f(0,1,0));
	mStaticCam.setViewDirection(mStaticCam_direction);
	
	mParams = params::InterfaceGl::create(this->getWindow(), "Parameters", Vec2i(350, 600));
//	mParams.setOptions("", "");
	mParams->addParam("fps: ", &mFps);
	vector<string> mesh_types;
	mesh_types.push_back("SurfaceTriMesh");
	mesh_types.push_back("SurfaceVboMesh");
	mParams->addParam("surface type", mesh_types, &mSurfaceMeshType);
	vector<string> cameras;
	cameras.push_back("Maya cam");
	cameras.push_back("Static cam");
	mParams->addParam("camera", cameras, &mCameraIndex);
	mParams->addParam("draw bezier patch latice", &mDrawBezierPatch);
	mParams->addParam("draw wireframe", &mDrawWireframe);
	mParams->addParam("enable backface culling", &mEnableBackfaceCulling);
	mParams->addParam("use additive blending", &mEnableAdditiveBlending);
	mParams->addParam("pause", &mPause);
	//	mParams->addParam("total hull edges", &mHullEdges);
	mParams->addParam("latice width", &mLaticeWidth);
	mParams->addParam("latice length", &mLaticeLength);
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
	mLightPosition = Vec3f(0, y * 4.0f, 3.0 - x * 6.0f);
}

void RibbonApp::mouseDrag( MouseEvent event )
{
	mMousePos = event.getPos();
	mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void RibbonApp::mouseUp( MouseEvent event )
{
}

void RibbonApp::mouseDown( MouseEvent event )
{
	mMayaCam.mouseDown( event.getPos() );
}

void RibbonApp::resize()
{
//	mCam.setPerspective( 60, getWindowAspectRatio(), 1, 1000 );
//	gl::setMatrices( mCam );
//	CameraPersp cam = mMayaCam.getCamera();
	mCam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam(mCam);
}

void RibbonApp::keyDown( KeyEvent event )
{
	if (event.isMetaDown() && event.getCode() == KeyEvent::KEY_w) this->quit();	// close window behavior
	
	Vec3f p;
	switch(event.getCode()){
		case KeyEvent::KEY_a:
//			rotate_b = false;
//			console()  << "rotation_y = " << rotation_y << std::endl;
			break;
		case KeyEvent::KEY_s:
//			rotate_b = true;
			break;
			
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
			mCam.setCenterOfInterestPoint(surfaceMesh.trimesh().calcBoundingBox().getCenter());
			mMayaCam.setCurrentCam(mCam);
			break;

//		case KeyEvent::KEY_UP:
//			p = mCam.getCenterOfInterestPoint();
//			p.y += 0.1f;
//			mCam.setCenterOfInterestPoint(p);
//			mMayaCam.setCurrentCam(mCam);
//			break;
//			
//		case KeyEvent::KEY_DOWN:
//			p = mCam.getCenterOfInterestPoint();
//			p.y -= 0.1f;
//			mCam.setCenterOfInterestPoint(p);
//			mMayaCam.setCurrentCam(mCam);
//			break;
//			
//		case KeyEvent::KEY_LEFT:
//			p = mCam.getCenterOfInterestPoint();
//			p.x -= 0.1f;
//			mCam.setCenterOfInterestPoint(p);
//			mMayaCam.setCurrentCam(mCam);
//			break;
//			
//		case KeyEvent::KEY_RIGHT:
//			p = mCam.getCenterOfInterestPoint();
//			p.x += 0.1f;
//			mCam.setCenterOfInterestPoint(p);
//			mMayaCam.setCurrentCam(mCam);
//			break;
		
		default:
			break;
	}
}

void RibbonApp::update()
{
	if (!mIsReady) return;
	
	mFps = static_cast<int32_t>(math<float>::floor(app::App::get()->getAverageFps()));
	
	if (!mPause) {
		this->updateSplinePatch();
		
		bsplineRect.create(mCtrlPoints, mLaticeWidth, mLaticeLength, mSplineDegreeU, mSplineDegreeV, mLoopU, mLoopV, mOpenU, mOpenV);
//		bsplineRect.setControlPointLatice(mCtrlPoints);	// does not work yet :(
		if (mSurfaceMeshType == 0 && surfaceMesh) {
			surfaceMesh = SurfaceTriMesh(bsplineRect, mMeshWidth, mMeshLength, Vec2f(0,0), Vec2f(1,1));
			surfaceMesh.updateSurface(bsplineRect);
		}
		else if (mSurfaceMeshType == 1 && surfaceVbo) {
			surfaceVbo = SurfaceVboMesh(bsplineRect, mMeshWidth, mMeshLength, Vec2f(0,0), Vec2f(1,1));
			surfaceVbo.updateSurface(bsplineRect);
		}
	}
	
	if (mCameraIndex == 1) {
		mStaticCam.setEyePoint(mStaticCam_position);
		mStaticCam.setOrtho(mStaticCam_left, mStaticCam_right, mStaticCam_bottom, mStaticCam_top, 1.0, 1000.0f);
		mStaticCam.setWorldUp(Vec3f(0,1,0));
		mStaticCam.setViewDirection(mStaticCam_direction);
	}
}

void RibbonApp::draw()
{
	if (!mIsReady) return;
	
	gl::clear(mClearColor, true);
//	gl::enableDepthTest();
	if (mEnableAdditiveBlending) {
		gl::enableAdditiveBlending();
	}
	else {
		gl::enableAlphaBlending();
	}
	gl::pushMatrices();
	if (mCameraIndex == 0) {
		gl::setMatrices(mMayaCam.getCamera());
	}
	else if (mCameraIndex == 1) {
		gl::setMatrices(mStaticCam);
	}
	
	if (mEnableBackfaceCulling) gl::enableBackFaceCulling();
	
//	drawWorldFrame();
	
	if (mDrawWireframe) gl::enableWireframe();
	else if (mShader) {
		gl::color(1,1,1,1);
		mShader.bind();
		mShader.uniform("lightPosition", Vec4f(mLightPosition,1.0f));
		mShader.uniform("eyePosition", mMayaCam.getCamera().getEyePoint());
		mShader.uniform("alpha", mAlpha);
		mShader.uniform("ambientColor", mAmbientColor);
		mShader.uniform("diffuseColor", mDiffuseColor);
		mShader.uniform("specularColor", mSpecularColor);
	}
	
	if (mSurfaceMeshType == 0 && surfaceMesh) {
		gl::draw(surfaceMesh);
	}
	else if (mSurfaceMeshType == 1 && surfaceVbo) {
		gl::draw(surfaceVbo);
	}
	
	gl::disableWireframe();
	
	gl::disableCulling();
	
	if (!mDrawWireframe && mShader) {
		mShader.unbind();
	}
	
	if (mDrawBezierPatch) {
		glPointSize(5.0f);
		gl::begin(GL_POINTS);
		gl::color(1,1,1,1);
		uint32_t i,j;
		for(i = 0; i != mLaticeWidth; ++i) {
			for(j = 0; j != mLaticeLength; ++j) {
				gl::vertex(mCtrlPoints[i + (j * mLaticeWidth)]);
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
	gl::drawLine(Vec3f::zero(), Vec3f(s,0,0));
	gl::color(0,1,0,1);
	gl::drawLine(Vec3f::zero(), Vec3f(0,s,0));
	gl::color(0,0,1,1);
	gl::drawLine(Vec3f::zero(), Vec3f(0,0,s));
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
	/*
	float offsets[24] = {0.9, 1.15, 1.75, 2.1, 2.25, 3.0, 2.9, 3.15, 5.05, 0.5, 0.555, 0.425,
						 4.5, 4.175, 3.8, 3.67, 3.6, 2.45, 2.415, 2.8, 2.375, 2.35, 1.95, 2.179};
	 */
	float weights[24] = {0.1, 0.115, 0.175, 0.33, 0.25, 0.275, 0.33, 0.39, 0.43, 0.5, 0.54, 0.42,
						0.5, 0.575, 0.6, 0.67, 0.6, 0.45, 0.415, 0.4, 0.375, 0.35, 0.15, 0.179};
	
	
	// mLaticeWidth =>	x axis
	// mLaticeLength =>	z axis
	// TODO: Modify the weights above to smooth it out
	if (mLoopV) {
		mCtrlPoints.resize(mLaticeWidth * mLaticeLength);
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
				Vec2f hull_pt = fromPolar(Vec2f(radius, theta));
				x = i * scale_x;
				y = hull_pt.y * scale_y;
				y += math<float>::cos((weights[j]) + i + t * freqX) * aX;
				y += math<float>::sin((t * freqZ) + (i * 0.75f));
				z = hull_pt.x * scale_z;
				mCtrlPoints[i + (j * mLaticeWidth)] = Vec3f(x,y,z);
				//mCtrlPoints.push_back(Vec3f(x,y,z));
				
				theta += t_x;
			}
		}
	}
	else {
		mCtrlPoints.resize(mLaticeWidth * mLaticeLength);
		uint32_t i,j;
		float fX,fY,aX, x,y,z;
		for(i = 0; i != mLaticeWidth; ++i) {
			for(j = 0; j != mLaticeLength; ++j) {
				fX = freqX;
				fY = j * t * math<float>::sin(0.15f * weights[j]);
//				float fX = freqX + (0.01f * static_cast<float>(j));
				aX = (0.75f * ampX) + (3.5f * weights[j]);// + (0.01f * static_cast<float>(j));
				x = i * scale_x;
				y = scale_y * math<float>::cos((1.0f * weights[j]) + i + t * fX) * aX;
				z = (j * scale_z) * (1.0f + math<float>::cos(i + t * freqY) * ampY);		//! position point-z
				y += (1.0 * weights[j]) + math<float>::sin(fY * freqZ) * ampZ;		//! add torsion-x to y
				mCtrlPoints[i + (j * mLaticeWidth)] = Vec3f(x,y,z);
				//mCtrlPoints.push_back(Vec3f(x,y,z));
			}
		}
	}
}

CINDER_APP_NATIVE( RibbonApp, RendererGl )

