#include <time.h>


#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/norm.hpp"
#include "glm/geometric.hpp"

#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/BSpline.h"
#include "cinder/Camera.h"
#include "cinder/CinderGlm.h"
#include "cinder/Color.h"
#include "cinder/GeomIo.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include "cinder/System.h"
#include "cinder/TriMesh.h"
#include "cinder/Utilities.h"
#include "cinder/Xml.h"

#include "BSplinePatch.h"
#include "BSplineSurface.h"

const static std::string rockWallNormalsImage = "images/rockwall_normals.png";
const static std::string rockWallImage = "images/rockwall.png";
const static std::string stoneBrickNormalsImage = "images/stone_brick_normals.png";
const static std::string stoneBrickWallImage = "images/stone_brick.png";
const static std::string texturedPhongFragShader = "shaders/TexturedPhong.frag";
const static std::string texturedPhongVertShader = "shaders/TexturedPhong.vert";

using namespace ci;
using namespace ci::app;
using namespace std;

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
	
	void pointSize(float size)
	{
		glPointSize(size);
	}
}
}

ci::vec2 objectToViewport( const ci::vec3& pt, const ci::mat4& composed, const ci::vec4& viewport )
{
	// get into clip space ...
	ci::vec4 x = ci::vec4( pt, 1.0f );
	ci::vec4 b = composed * x;
	// then into normalize device space ...
	if (b.w != 0.0f) b.w = 1.0f / b.w;
	b.x *= b.w;
	b.y *= b.w;
	b.z *= b.w;
	
	// now into window/viewport space ...
	ci::vec2 result;
	result.x = viewport.x + viewport.z * (b.x + 1.0f) / 2.0f;
	result.y = viewport.y + viewport.w * (1.0f - (b.y + 1.0f) / 2.0f);
	
	return result;
}

class BSplineTestApp : public AppNative {
public:
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
	void generateControlPoints( const ivec2& size = ivec2(5), const vec2& scale = vec2(1) );
	
	CameraPersp				mCam;
	MayaCamUI				mMayaCam;
	ivec2					mMousePos;
	
	float					mFps;
	bool					mIsMouseDown;
	bool					mDrawBezierPatch;
	bool					mDrawWireframe;
	bool					mEnableBackfaceCulling;
	bool					mEnableAdditiveBlending;
	bool					mLoopU;
	bool					mLoopV;
	bool					mOpenU;
	bool					mOpenV;
	float					mAlpha;
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
	params::InterfaceGlRef	mParams;
	
	bool					mIsReady;
	BSplinePatch			mBSplineRect;
	std::vector<ci::vec3>	mCtrlPoints;
	ci::vec3*				mSelectedCtrlPt;
	ci::mat4				mMVP;
	
	ci::gl::TextureRef		mTextureMap;
	ci::gl::TextureRef		mNormalMap;
	ci::gl::VboMeshRef		mSurfaceMesh;
	ci::gl::BatchRef		mRenderBatch;
	ci::gl::GlslProgRef		mShader;
};

void BSplineTestApp::prepareSettings( Settings *settings )
{
	mIsReady = false;
	
	settings->setWindowSize(1600, 1080);
	settings->setFrameRate(60);
	settings->setTitle("BSplineTestApp");
	
	// seed random number generators
	srand(time(NULL));
	Rand::randSeed(time(NULL));
}

void BSplineTestApp::setup()
{
	// Add Mac resource paths...
	fs::path path;
	path = this->getAppPath() / ".." / ".." / ".." / ".." / "assets";
	this->addAssetDirectory(path);
	
	this->getWindow()->getSignalResize().connect(std::bind(&BSplineTestApp::resize, this));
	
	mSelectedCtrlPt = nullptr;
	
	// initialize application data
	mFps = 0.0;
	mEnableBackfaceCulling = false;
	mEnableAdditiveBlending = false;
	mDrawBezierPatch = true;
	mDrawWireframe = true;
	mLaticeWidth = 5;
	mLaticeLength = 5;
	mMeshWidth = 30;
	mMeshLength = 30;
	mSplineDegreeU = 3;
	mSplineDegreeV = 3;
	mLoopU = false;
	mLoopV = false;
	mOpenU = true;
	mOpenV = true;
	mAlpha = 0.6f;
	mAmbientColor = Color(30.0/255.0, 80.0/255.0, 90.0/255.0);
	mDiffuseColor = Color(56.0/255.0, 120.0/255.0, 110.0/255.0);
	mSpecularColor = Color(216.0/255.0, 252.0/255.0, 238.0/255.0);
	mClearColor = ColorA(8.0f/255.0, 26.0/255.0, 42.0/255.0, 1.0);
	mLightPosition = vec3( 0, 10.0f, 0 );
	
	try {
		mShader = gl::GlslProg::create(app::loadAsset(texturedPhongVertShader), app::loadAsset(texturedPhongFragShader));
		gl::Texture::Format txtfmt;
		txtfmt.mipmap(true);
		txtfmt.wrap( GL_REPEAT );
		mTextureMap = gl::Texture::create( ci::loadImage( app::loadAsset(rockWallImage) ), txtfmt );
		mNormalMap = gl::Texture::create( ci::loadImage( app::loadAsset(rockWallNormalsImage) ), txtfmt );
	}
	catch ( Exception& exc ) {
		console() << "Asset error: " << exc.what() << std::endl;
	}
	
	// construct bspline mesh and batch
	generateControlPoints( ivec2(mLaticeWidth, mLaticeLength) );
	mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
	BSplineSurface surfaceMesh = BSplineSurface( mBSplineRect, ivec2(mMeshWidth, mMeshLength) ).texCoords( vec2(0,0), vec2(1,1) );
	mSurfaceMesh = gl::VboMesh::create( surfaceMesh );
	mRenderBatch = gl::Batch::create( surfaceMesh, mShader );
	
	// configure camera
	mCam.setEyePoint( vec3( 4.40f, 2.75f, 5.75f ) );
	mCam.setCenterOfInterestPoint( vec3( 2.5f, 0, 2.5f ) );
	mCam.setWorldUp( vec3( 0, 1, 0 ) );
	mCam.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
	mMayaCam.setCurrentCam(mCam);
	
	// configure parameters
	mParams = params::InterfaceGl::create(this->getWindow(), "Parameters", ivec2(350, 600));
	mParams->addParam("fps: ", &mFps);
	mParams->addSeparator();
	mParams->addParam("draw bezier patch latice", &mDrawBezierPatch);
	mParams->addParam("draw wireframe", &mDrawWireframe);
	mParams->addParam("enable backface culling", &mEnableBackfaceCulling);
	mParams->addParam("use additive blending", &mEnableAdditiveBlending);
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
	mParams->addSeparator();
	mParams->addText("Shader parameters");
	mParams->addParam("alpha transparency", (float*)&mAlpha, "min=0.0 max=1.0");
	mParams->addParam("ambient color", &mAmbientColor);
	mParams->addParam("diffuse color", &mDiffuseColor);
	mParams->addParam("specular color", &mSpecularColor);
	mParams->addParam("clear color", &mClearColor);
	
	gl::enableDepthRead();
	gl::enableDepthWrite();
	
	mIsReady = true;
}

void BSplineTestApp::mouseMove( MouseEvent event )
{
	mMousePos = event.getPos();
}

void BSplineTestApp::mouseDrag( MouseEvent event )
{
	mMousePos = event.getPos();
	
	if (event.isMetaDown()) {
		mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
	}
	else if (event.isAltDown()) {
		vec2 pos = vec2( mMousePos );
		vec2 window = vec2( getWindowSize() );
		vec2 light = pos / window;
		mLightPosition = vec3( 5.0f - light.y * 15.0f, 0, 5.0f - light.x * 15.0f );
	}
	else if (nullptr != mSelectedCtrlPt) {
		CameraPersp cam = mMayaCam.getCamera();
		vec2 pos = vec2( mMousePos );
		vec2 offset = gl::getViewport().first;
		vec2 size = gl::getViewport().second;
		vec4 viewport = vec4( offset.x, offset.y, size.x, size.y );
		
		Ray r = cam.generateRay( pos.x/viewport.z, (viewport.w - pos.y)/viewport.w, cam.getAspectRatio() );
		
		vec3 n( cam.getEyePoint() );
		normalize(n);
		vec3 dVector = *mSelectedCtrlPt;
		float distance = -dot(dVector, n);
		vec3 origin = r.getOrigin();
		vec3 direction = r.getDirection();
		
		// calculate intersection point.
		//t = -(AX0 + BY0 + CZ0 + D) / (AXd + BYd + CZd)
		float t_hit = -(n.x*origin.x + n.y*origin.y + n.z*origin.z + distance) / (n.x*direction.x + n.y*direction.y + n.z*direction.z);
		vec3 intersection = origin + (direction * t_hit);
		*mSelectedCtrlPt = intersection;
	}
}

void BSplineTestApp::mouseUp( MouseEvent event )
{
	mIsMouseDown = false;
}

void BSplineTestApp::mouseDown( MouseEvent event )
{
	mIsMouseDown = true;
	
	mMousePos = event.getPos();
	
	if (event.isMetaDown()) {
		mMayaCam.mouseDown( mMousePos );
	}
	else {
		mSelectedCtrlPt = nullptr;
		
		const float click_dist_tol = 5.0f;
		mat4 model = mat4(1.0);
		mat4 view = mMayaCam.getCamera().getViewMatrix();
		mat4 projection = mMayaCam.getCamera().getProjectionMatrix();
		vec2 offset = gl::getViewport().first;
		vec2 size = gl::getViewport().second;
		vec4 viewport = vec4( offset.x, offset.y, size.x, size.y );
		for (size_t i=0; i<mCtrlPoints.size(); i++) {
			vec3& pt = mCtrlPoints[i];
			mat4 mvp = projection * view * model;
			vec2 screenspace_pt = objectToViewport( pt, mvp, viewport );
			float pt_distance = ci::distance( vec2(mMousePos), screenspace_pt);
			if (click_dist_tol > pt_distance) {
				mSelectedCtrlPt = &mCtrlPoints[i];
				break;
			}
		}
	}
}

void BSplineTestApp::resize()
{
	mCam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam(mCam);
}

void BSplineTestApp::keyDown( KeyEvent event )
{
	if (event.isMetaDown() && event.getCode() == KeyEvent::KEY_w) this->quit();
	
	switch(event.getCode()){
		case KeyEvent::KEY_f:
			setFullScreen(!isFullScreen());
			break;
			
		case KeyEvent::KEY_DOWN:
			if (mSelectedCtrlPt) mSelectedCtrlPt->y -= 0.1f;
			break;
		case KeyEvent::KEY_UP:
			if (mSelectedCtrlPt) mSelectedCtrlPt->y += 0.1f;
			break;
		case KeyEvent::KEY_LEFT:
			if (mSelectedCtrlPt) mSelectedCtrlPt->z -= 0.1f;
			break;
		case KeyEvent::KEY_RIGHT:
			if (mSelectedCtrlPt) mSelectedCtrlPt->z += 0.1f;
			break;
			
		case KeyEvent::KEY_p:
			{
				mat4 view = mMayaCam.getCamera().getViewMatrix();
				mat4 projection = mMayaCam.getCamera().getProjectionMatrix();
				console() << "view: " << view << std::endl;
				console() << "projection: " << projection << std::endl;
			}
			break;
			
		default:
			break;
	}
}

void BSplineTestApp::update()
{
	if (!mIsReady) return;
	
	mFps = static_cast<int32_t>(math<float>::floor(app::App::get()->getAverageFps()));
	
	mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
	BSplineSurface surfaceMesh = BSplineSurface( mBSplineRect, ivec2(mMeshWidth, mMeshLength) ).texCoords( vec2(0,0), vec2(1,1) );
	mSurfaceMesh = gl::VboMesh::create( surfaceMesh );
	mRenderBatch = gl::Batch::create( mSurfaceMesh, mShader );
}

void BSplineTestApp::draw()
{
	if (!mIsReady) return;
	
	gl::clear( mClearColor, true );
	gl::color( 1,1,1,1 );
	
	if (mEnableAdditiveBlending) {
		gl::enableAdditiveBlending();
	}
	else {
		gl::enableAlphaBlending();
	}
	gl::pushMatrices();
	gl::setMatrices( mMayaCam.getCamera() );
	
	drawWorldFrame();
	
	if (mEnableBackfaceCulling) {
		gl::enableBackFaceCulling();
	}
	
	if (mDrawWireframe) {
		gl::color( 1,1,1,1 );
		gl::enableWireframe();
		gl::draw( mSurfaceMesh );
		gl::disableWireframe();
	}
	else {	
		mTextureMap->bind(0);
		mShader->uniform( "uTex0", 0 );
		mShader->uniform( "lightPosition", mLightPosition );
		mRenderBatch->draw();
		mTextureMap->unbind();
	}
	
	if (mEnableBackfaceCulling) {
		gl::disableCulling();
	}
	
	if (mDrawBezierPatch) {
		gl::pointSize( 5.0f );
		gl::begin( GL_POINTS );
		gl::color( 0,1,1,1 );
		uint32_t i,j;
		for(i = 0; i != mLaticeWidth; ++i) {
			for(j = 0; j != mLaticeLength; ++j) {
				if (mSelectedCtrlPt == &mCtrlPoints[i * mLaticeLength + j]) {
					gl::color( 1,0,0,1 );
					gl::vertex( mCtrlPoints[i * mLaticeLength + j] );
					gl::color( 0,1,1,1 );
				}
				else {
					gl::vertex( mCtrlPoints[i * mLaticeLength + j] );
				}
			}
		}
		
		gl::end();
	}
	
	gl::popMatrices();
	
	if (false) {
		gl::color( 1,0,1,1 );
		if (mIsMouseDown) {
			gl::drawStrokedCircle( mMousePos, 10.0f, 16.0f );
		}
		
		mat4 model = mat4(1.0);
		mat4 view = mMayaCam.getCamera().getViewMatrix();
		mat4 projection = mMayaCam.getCamera().getProjectionMatrix();
		vec2 offset = gl::getViewport().first;
		vec2 size = gl::getViewport().second;
		vec4 viewport = vec4( offset.x, offset.y, size.x, size.y );
		for (size_t i=0; i<mCtrlPoints.size(); i++) {
			vec3& pt = mCtrlPoints[i];
			mat4 mvp = projection * view * model;
			vec2 screenspace_pt = objectToViewport( pt, mvp, viewport );
			gl::drawStrokedCircle( screenspace_pt, 5.0f, 16.0f );
		}
		gl::color( 1,1,1,1 );
	}
	
	mParams->draw();
}

void BSplineTestApp::drawWorldFrame()
{
	float s = 1.0f;
	gl::color(1,0,0,1);
	gl::drawLine(vec3(0), vec3(s,0,0));
	gl::color(0,1,0,1);
	gl::drawLine(vec3(0), vec3(0,s,0));
	gl::color(0,0,1,1);
	gl::drawLine(vec3(0), vec3(0,0,s));
}

void BSplineTestApp::generateControlPoints( const ivec2& size, const vec2& scale )
{
	mCtrlPoints.resize(size.x * size.y);
	for (size_t i = 0; i < size.x; i++) {
		for (size_t j = 0; j < size.y; j++) {
			mCtrlPoints[i * size.y + j] = vec3(i * scale.x, 0, j * scale.y);
		}
	}
}

CINDER_APP_NATIVE( BSplineTestApp, RendererGl )