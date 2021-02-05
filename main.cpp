#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Arcball.h"
#include "cinder/Rand.h"
#include "cinder/ObjLoader.h"
#include "cinder/CameraUi.h"

#include "include/Resources.h"

#include <chrono>
#include <sstream>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <msgpack.hpp>
#include <json.hpp>

using namespace ci;
using namespace ci::app;

using json = nlohmann::json;
using namespace std::chrono_literals;

class GazeTracking : public App
{
public:
	void setup();
	void keyDown(KeyEvent event) override;
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;
	void mouseWheel(MouseEvent event) override;
	void mouseUp(MouseEvent event) override;
	void resize() override;
	void update() override;
	void draw() override;
private:
	CameraPersp cam_;
	CameraUi cameraUi_{&cam_};
	TriMeshRef arrowMesh_;
	gl::BatchRef arrow_;
	
	std::vector<TriMeshRef> meshes_;
	std::vector<gl::BatchRef> batches_;

	gl::TextureRef textTexture_;
	Font font_{"Arial", 24};
	vec3 pos_;
	quat rot_;
	vec3 gazePoint_{0.0f, 0.0f, 1.0f};
	rs2::pipeline pipeline_;
	zmq::context_t context_;
	zmq::socket_t sub_socket_;
	bool mouseDown_{false};
	bool useGaze_{true};
};

void prepareSettings(GazeTracking::Settings *settings)
{
	settings->setMultiTouchEnabled(false);
	settings->setWindowSize(2400, 1600);
}

void GazeTracking::setup()
{
	cam_.setPerspective(60, getWindowWidth() / static_cast<float>(getWindowHeight()), 1, 1000);
	cam_.lookAt({0.0f, 0.0f, 10.0f}, {0.0f, 0.0f, 0.0f});

	auto lambert = gl::ShaderDef().lambert().color();
	auto shader = gl::getStockShader(lambert);

	arrowMesh_ = TriMesh::create(
		(geom::Cone() & (geom::Cylinder().radius(0.2f) >> geom::Translate(0.0f, -2.0f, 0.0f))) >> geom::Rotate(-M_PI / 2.0f, {1.0f, 0.0f, 0.0f}));

	arrow_ = gl::Batch::create(*arrowMesh_, shader);

	ObjLoader loader( loadAsset("cone.obj") );
	auto mesh = TriMesh::create( loader );
	meshes_.push_back(mesh);
	batches_.push_back(gl::Batch::create(*mesh, shader));

	loader = loadAsset("monkey.obj");
	mesh = TriMesh::create( loader );
	meshes_.push_back(mesh);
	batches_.push_back(gl::Batch::create(*mesh, shader));
	

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	pipeline_.start(cfg);

	// construct a REQ (request) socket and connect to interface
	zmq::socket_t socket{context_, zmq::socket_type::req};
	socket.connect("tcp://localhost:50020");

	// set up some static data to send
	const std::string sub_request{"SUB_PORT"};
	socket.send(zmq::buffer(sub_request), zmq::send_flags::dontwait);

	std::this_thread::sleep_for(100ms);

	bool pupil_detected = true;
	zmq::message_t reply{};
	auto res = socket.recv(reply, zmq::recv_flags::dontwait);
	if (!res)
	{
		pupil_detected = false;
	}

	if (pupil_detected)
	{
		std::string sub_port = reply.to_string();

		const std::string pub_request{"PUB_PORT"};
		socket.send(zmq::buffer(pub_request), zmq::send_flags::none);
		// wait for reply from server
		socket.recv(reply, zmq::recv_flags::none);
		std::string pub_port = reply.to_string();

		sub_socket_ = zmq::socket_t{context_, zmq::socket_type::sub};
		sub_socket_.connect(std::string("tcp://localhost:") + sub_port);
		sub_socket_.set(zmq::sockopt::subscribe, "gaze.");
	}

	
}

void GazeTracking::resize()
{
	cam_.setPerspective(
		60, getWindowWidth() / static_cast<float>(getWindowHeight()), 1, 1000);
	cam_.lookAt({0.0f, 0.0f, 10.0f}, {0.0f, 0.0f, 0.0f});
}

void GazeTracking::mouseDown(MouseEvent event)
{
	cameraUi_.mouseDown(event);
	mouseDown_ = true;
}

void GazeTracking::mouseDrag(MouseEvent event)
{
	cameraUi_.mouseDrag(event);
}

void GazeTracking::mouseUp(MouseEvent event)
{
	cameraUi_.mouseUp(event);
	mouseDown_ = false;
}

void GazeTracking::mouseWheel(MouseEvent event)
{
	cameraUi_.mouseWheel(event);
}

void GazeTracking::keyDown(KeyEvent event)
{
	if (event.getChar() == 'f')
	{
		setFullScreen(!isFullScreen());
	}
	else if (event.getChar() == 'g')
	{
		useGaze_ = ! useGaze_;
	}
	else if (event.getCode() == KeyEvent::KEY_SPACE)
	{
	}
	else if (event.getCode() == KeyEvent::KEY_ESCAPE)
	{

		if (isFullScreen())
			setFullScreen(false);
		else
			quit();
	}
}

void GazeTracking::update()
{
	if (sub_socket_)
	{
		std::vector<zmq::message_t> recv_msgs;
		zmq::recv_result_t result = zmq::recv_multipart(sub_socket_, std::back_inserter(recv_msgs));
		msgpack::object_handle oh = msgpack::unpack((const char *)recv_msgs[1].data(), recv_msgs[1].size());
		msgpack::object obj = oh.get();
		std::ostringstream oss;
		oss << obj;
		auto j = json::parse(oss.str());

		auto confidence = j["confidence"];
		auto gaze_point_3d = j["gaze_point_3d"];

		float c = static_cast<float>(confidence);

		if (c > 0.9f) {
			float x = static_cast<float>(gaze_point_3d[0]);
			float y = static_cast<float>(gaze_point_3d[1]);
			float z = static_cast<float>(gaze_point_3d[2]);

			// mm -> m
			// change coordinate system
			if (useGaze_) {
				gazePoint_ = {x/100.0f, -y/100.0f, -z/100.0f};
			} else {
				gazePoint_ = {0.0f, 0.0f, -1.0f};
			}
		}
	}

	rs2::frameset frameset;

	if (pipeline_.poll_for_frames(&frameset))
	{
		if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
		{
			rs2_pose pose_sample = pose_frame.get_pose_data();
			pos_ = vec3{pose_sample.translation.x, pose_sample.translation.y, pose_sample.translation.z};
			rot_ = {pose_sample.rotation.w, pose_sample.rotation.x, pose_sample.rotation.y, pose_sample.rotation.z};
		}
	}

}

void GazeTracking::draw()
{
	{
		gl::ScopedMatrices scope;

		gl::clear(Color::gray(0.1f));
		gl::color(1.0f, 0.5f, 0.25f);
		gl::enableDepthRead();
		gl::enableDepthWrite();
		gl::setMatrices(cam_);

		// vec3 gazePoint = {gazePoint_.x, -gazePoint_.y, -gazePoint_.z};
		vec3 gazePoint = {gazePoint_.x, gazePoint_.y, gazePoint_.z};
		vec3 gazeVec = normalize(gazePoint);
		vec3 rotatedGazeVec = rot_*gazeVec;

		quat eyeQuat = rotation({0.0f, 0.0f, -1.0f}, gazeVec);

		gl::drawCoordinateFrame(5.0f);
		// room_->draw();
		for (auto& batch : batches_) {
			batch->draw();
		}

		Ray ray{pos_, rotatedGazeVec};

		TextBox tbox = TextBox().alignment(TextBox::LEFT).font(font_).size(ivec2(600, TextBox::GROW)).text("");
		tbox.setColor(Color(1.0f, 0.65f, 0.35f));
		tbox.setBackgroundColor(ColorA(0.5, 0, 0, 1));
		textTexture_ = gl::Texture2d::create(tbox.render());


		int id = 0;

		for (auto& mesh : meshes_) {
			const size_t trianglecount = mesh->getNumTriangles();

			float result = FLT_MAX;
			float distance = 0.0f;

			for( size_t i = 0; i < trianglecount; ++i ) {
				vec3 v0, v1, v2;
				mesh->getTriangleVertices( i, &v0, &v1, &v2 );

				if( ray.calcTriangleIntersection( v0, v1, v2, &distance ) ) {
					auto pickedNormal = normalize( cross( v1 - v0, v2 - v0 ) );
					float d = dot(pickedNormal, ray.getDirection());

					if (distance > 0.0f && d < 0.0f) {
						if( distance < result ) {
							result = distance;
						}
					}
				}
			}

			// Did we have a hit?
			if( distance > 0.0f ) {
				// Calculate the exact position of the hit.
				auto pickedPoint = ray.calcPosition( result );
				gl::ScopedColor color(0.0f, 0.0f, 1.0f, 0.5f);
				gl::drawCube(pickedPoint, {2.0f, 2.0f, 2.0f});

				std::ostringstream oss_gp;
				oss_gp << "Object " << id << std::endl;
				TextBox tbox = TextBox().alignment(TextBox::LEFT).font(font_).size(ivec2(600, TextBox::GROW)).text(oss_gp.str());
				tbox.setColor(Color(1.0f, 0.65f, 0.35f));
				tbox.setBackgroundColor(ColorA(0.5, 0, 0, 1));
				textTexture_ = gl::Texture2d::create(tbox.render());
			}

			id++;

		}

		gl::drawLine(pos_, pos_ + 100.0f*rotatedGazeVec);

		{
			gl::ScopedMatrices scope;
			gl::translate(pos_.x, pos_.y, pos_.z);
			gl::rotate(rot_ * eyeQuat);
			arrow_->draw();
		}
	}

	if (textTexture_)
		gl::draw(textTexture_);
}

CINDER_APP(GazeTracking, RendererGl, prepareSettings)
