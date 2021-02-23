#include <vector>
#include <string>
#include <functional>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/norm.hpp"
#include "glm/geometric.hpp"

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/BSpline.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/CinderGlm.h"
#include "cinder/Color.h"
#include "cinder/GeomIo.h"
#include "cinder/ImageIo.h"
#include "cinder/Log.h"
#include "cinder/Rand.h"
#include "cinder/System.h"
#include "cinder/TriMesh.h"
#include "cinder/Utilities.h"
#include "cinder/Xml.h"

#include "BSplinePatch.h"
#include "BSplineSurface.h"

const std::string rockWallNormalsImage = "images/rockwall_normals.png";
const std::string rockWallImage = "images/rockwall.png";
const std::string stoneBrickNormalsImage = "images/stone_brick_normals.png";
const std::string stoneBrickWallImage = "images/stone_brick.png";
const std::string texturedPhongFragShader = "shaders/TexturedPhong.frag";
const std::string texturedPhongVertShader = "shaders/TexturedPhong.vert";

using namespace ci;
using namespace ci::app;
using namespace std;

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

class BSplineTestApp : public App {
public:
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
	void updateSplineSurface();
	void generateSplineSurface();
	int generateControlPoints_plane( const ivec2& size = ivec2(5), const vec2& scale = vec2(1) );
	int generateControlPoints_cylinder( float radius = 1.0f, float height = 2.0f, const ivec2& subd = ivec2(10,4) );
	int generateControlPoints_cube( const ivec3& size = ivec3(2), const vec3& scale = vec3(2) );
	
	int32_t getPatchWidth();
	void setPatchWidth(int32_t width);
	int32_t getPatchLength();
	void setPatchLength(int32_t length);
	
	CameraPersp				mCam;
	CameraUi				mMayaCam;
	ivec2					mMousePos;
	DisplayRef				mWindowDisplay;
	
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
	
	bool					mIsReady = { false };
	BSplinePatch			mBSplineRect;
	std::vector<ci::vec3>	mCtrlPoints;
	ci::vec3*				mSelectedCtrlPt;

	std::vector<std::string> mSplineModes;
	int32_t					mSplineIndex;
	
	ci::gl::TextureRef		mTextureMap;
	ci::gl::TextureRef		mNormalMap;
	ci::gl::VboMeshRef		mSurfaceMesh;
	ci::gl::BatchRef		mRenderBatch;
	ci::gl::GlslProgRef		mShader;
};

void BSplineTestApp::setup()
{
	getWindow()->getSignalResize().connect(std::bind(&BSplineTestApp::resize, this));
	getWindow()->getSignalDisplayChange().connect(std::bind(&BSplineTestApp::resize, this));
	mWindowDisplay = getWindow()->getDisplay();
	
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
	mSplineDegreeU = 2;
	mSplineDegreeV = 2;
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
	mSplineModes = { "Plane", "Cylinder" };//, "Cube" };
	mSplineIndex = 0;
	
	try {
		mShader = gl::GlslProg::create(app::loadAsset(texturedPhongVertShader), app::loadAsset(texturedPhongFragShader));
		gl::Texture::Format txtfmt;
		txtfmt.mipmap(true);
		txtfmt.wrap( GL_REPEAT );
		mTextureMap = gl::Texture::create( ci::loadImage( app::loadAsset(rockWallImage) ), txtfmt );
		mNormalMap = gl::Texture::create( ci::loadImage( app::loadAsset(rockWallNormalsImage) ), txtfmt );
	}
	catch ( Exception& exc ) {
		CI_LOG_W( "Asset error: " << exc.what() );
	}
	
	generateSplineSurface();
	
	// configure camera
	mCam.setEyePoint( vec3( 4.40f, 2.75f, 5.75f ) );
	mCam.lookAt( vec3( 2.5f, 0, 2.5f ) );
	mCam.setWorldUp( vec3( 0, 1, 0 ) );
	mCam.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
	mMayaCam.setCamera(&mCam);
	
	// configure parameters
	mParams = params::InterfaceGl::create(this->getWindow(), "Parameters", ivec2(350, 600));
	mParams->addParam("fps: ", &mFps);
	mParams->addParam("Modes", mSplineModes, &mSplineIndex);
	mParams->addSeparator();
	mParams->addParam("draw bezier patch latice", &mDrawBezierPatch);
	mParams->addParam("draw wireframe", &mDrawWireframe);
	mParams->addParam("enable backface culling", &mEnableBackfaceCulling);
	mParams->addParam("use additive blending", &mEnableAdditiveBlending);
	mParams->addParam("patch width", &mLaticeWidth);
	mParams->addParam("patch length", &mLaticeLength);
//	mParams->addParam("patch width", std::bind(&BSplineTestApp::setPatchWidth, this, std::placeholders::_1), std::bind(&BSplineTestApp::getPatchWidth, this));
//	mParams->addParam("patch length", std::bind(&BSplineTestApp::setPatchLength, this, std::placeholders::_1), std::bind(&BSplineTestApp::getPatchLength, this));
	mParams->addParam("patch length", &mLaticeLength);
	mParams->addParam("mesh width", &mMeshWidth);
	mParams->addParam("mesh length", &mMeshLength);
	mParams->addParam("spline degree U", &mSplineDegreeU);
	mParams->addParam("spline degree V", &mSplineDegreeV);
	mParams->addParam("loop U", &mLoopU);
	mParams->addParam("loop V", &mLoopV);
	mParams->addParam("open U", &mOpenU);
	mParams->addParam("open V", &mOpenV);
	
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
	mMayaCam.setCamera(&mCam);
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
			
		default:
			break;
	}
}

void BSplineTestApp::update()
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
	
	static uint32_t lastSplineIndex = 10;
	static uint32_t lastPatchWidth = 0;
	static uint32_t lastPatchLength = 0;
	if (lastSplineIndex != mSplineIndex) {
		generateSplineSurface();
		lastSplineIndex = mSplineIndex;
	}
	else if (lastPatchWidth != mLaticeWidth ||
			 lastPatchLength != mLaticeLength) {
		generateSplineSurface();
		lastPatchWidth = mLaticeWidth;
		lastPatchLength = mLaticeLength;
	}
	else {
		updateSplineSurface();
	}
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
	
	gl::drawCoordinateFrame();
	
	if (mEnableBackfaceCulling) {
		gl::enableFaceCulling();
		gl::cullFace(GL_BACK);
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
		gl::enableFaceCulling( false );
	}
	
	if (mDrawBezierPatch) {
		gl::pointSize( 5.0f );
		gl::begin( GL_POINTS );
		gl::color( 0,1,1,1 );
		for (size_t i=0; i<mCtrlPoints.size(); i++) {
			if (mSelectedCtrlPt == &mCtrlPoints[i]) {
				gl::color( 1,0,0,1 );
				gl::vertex( mCtrlPoints[i] );
				gl::color( 0,1,1,1 );
			}
			else {
				gl::vertex( mCtrlPoints[i] );
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

void BSplineTestApp::updateSplineSurface()
{
	// construct bspline mesh and batch
	switch(mSplineIndex) {
		case 0:	// plane
			mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
			break;
		case 1:	// cylinder
			mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
			break;
		case 2:	// cube
		default:
			mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(2, 3), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
			break;
	}
	
	BSplineSurface surfaceMesh = BSplineSurface( const_cast<BSplinePatch*>(&mBSplineRect), ivec2(mMeshWidth, mMeshLength) ).texCoords( vec2(0,0), vec2(1,1) );
	mSurfaceMesh = gl::VboMesh::create( surfaceMesh );
	mRenderBatch = gl::Batch::create( surfaceMesh, mShader );
}

void BSplineTestApp::generateSplineSurface()
{
	// construct bspline mesh and batch
	switch(mSplineIndex) {
		case 0:	// plane
			mOpenV = mOpenU = true;
			mLoopV = mLoopU = false;
			generateControlPoints_plane( ivec2(mLaticeWidth, mLaticeLength) );
			mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
			break;
		case 1:	// cylinder
			mOpenU = mLoopV = false;
			mOpenV = mLoopU = true;
			generateControlPoints_cylinder( 1.0, 2.0, ivec2(mLaticeWidth, mLaticeLength) );
			mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(mLaticeWidth, mLaticeLength), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
			break;
		case 2:	// cube
		default:
			mOpenV = mOpenU = false;
			mLoopV = mLoopU = true;
			generateControlPoints_cube();
			mBSplineRect = BSplinePatch( mCtrlPoints, ivec2(3, 2), ivec2(mSplineDegreeU, mSplineDegreeV), glm::bvec2(mLoopU, mLoopV), mOpenU, mOpenV );
			break;
	}
	
	BSplineSurface surfaceMesh = BSplineSurface( const_cast<BSplinePatch*>(&mBSplineRect), ivec2(mMeshWidth, mMeshLength) ).texCoords( vec2(0,0), vec2(1,1) );
	mSurfaceMesh = gl::VboMesh::create( surfaceMesh );
	mRenderBatch = gl::Batch::create( surfaceMesh, mShader );
}

int BSplineTestApp::generateControlPoints_plane( const ivec2& size, const vec2& scale )
{
	mCtrlPoints.resize(size.x * size.y);
	for (size_t i = 0; i < size.x; i++) {
		for (size_t j = 0; j < size.y; j++) {
			mCtrlPoints[i * size.y + j] = vec3(i * scale.x, 0, j * scale.y);
		}
	}
	
	return mCtrlPoints.size();
}

int BSplineTestApp::generateControlPoints_cylinder( float radius, float height, const ivec2& subd )
{
	vec3 origin = vec3( 2.5, 0.0, 2.5 );
	vec3 direction = vec3( 0, 1, 0 );
	int mNumSegments = subd.s;
	int mNumSlices = subd.t;
	
	const float segmentIncr = 1.0f / mNumSegments;
	const float ringIncr = 1.0f / mNumSlices;
	const quat axis( vec3( 0, 1, 0 ), direction );
	
	// vertex, normal, tex coord and color buffers
	mCtrlPoints.clear();
	mCtrlPoints.reserve(mNumSegments * mNumSlices);
	for( size_t i = 0; i < mNumSegments; ++i ) {
		for( size_t j = 0; j < mNumSlices; ++j ) {
			float cosPhi = -ci::math<float>::cos( i * segmentIncr * float(M_PI * 2) );
			float sinPhi =  ci::math<float>::sin( i * segmentIncr * float(M_PI * 2) );
			float x = radius * cosPhi;
			float y = height * j * ringIncr;
			float z = radius * sinPhi;
			
			mCtrlPoints.emplace_back( origin + axis * vec3( x, y, z ) );
		}
	}
	
	return mCtrlPoints.size();
}

int BSplineTestApp::generateControlPoints_cube( const ivec3& size, const vec3& scale )
{
	CI_ASSERT_MSG( false, "Cube primitive not yet supported" );
	
	mCtrlPoints.clear();
	mCtrlPoints.resize(size.x * size.y * size.z);
//	for (size_t i = 0; i < size.x; i++) {
//		for (size_t j = 0; j < size.y; j++) {
//			for (size_t k = 0; k < size.z; k++) {
//				mCtrlPoints[i * size.y + j] = vec3(i * scale.x, 0, j * scale.y);
//			}
//		}
//	}
	
	float x = scale.x;
	float y = scale.y;
	float z = scale.z;
	mCtrlPoints[0] = vec3(0,  0,  0  );
	mCtrlPoints[1] = vec3(0,  0,  z  );
	mCtrlPoints[2] = vec3(x,  0,  0  );
	mCtrlPoints[3] = vec3(x,  0,  z  );
	
	mCtrlPoints[4] = vec3(0,  y,  z  );
	mCtrlPoints[5] = vec3(x,  y,  z  );
	mCtrlPoints[6] = vec3(0,  y,  0  );
	mCtrlPoints[7] = vec3(x,  y,  0  );
	
	return mCtrlPoints.size();
}

int32_t BSplineTestApp::getPatchWidth()
{
	return mLaticeWidth;
}

void BSplineTestApp::setPatchWidth(int32_t width)
{
	mLaticeWidth = width;
	generateSplineSurface();
}

int32_t BSplineTestApp::getPatchLength()
{
	return mLaticeLength;
}

void BSplineTestApp::setPatchLength(int32_t length)
{
	mLaticeLength = length;
	generateSplineSurface();
}

CINDER_APP( BSplineTestApp, RendererGl(),
[&]( App::Settings *settings ) {
	settings->setDisplay(Display::getMainDisplay());
	settings->setHighDensityDisplayEnabled( false );
	settings->setWindowSize(1600, 1080);
	settings->setFrameRate(60);
	settings->setTitle("BSplineTestApp");
	
	// seed random number generators
	srand(time(NULL));
	Rand::randSeed(time(NULL));
})
