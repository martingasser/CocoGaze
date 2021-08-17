#include <msgpack.hpp>

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Arcball.h"
#include "cinder/Rand.h"
#include "cinder/ObjLoader.h"
#include "cinder/CameraUi.h"
#include "cinder/CinderImGui.h"
#include "cinder/Json.h"

#include "include/Resources.h"

#include <chrono>
#include <sstream>
#include <thread>
#include <atomic>
#include <fstream>
#include <streambuf>

#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <json.hpp>

using namespace ci;
using namespace ci::app;

using json = nlohmann::json;
using namespace std::chrono_literals;

class GazeTracking : public App
{
public:
    ~GazeTracking();
	void setup() override;
	void keyDown(KeyEvent event) override;
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;
	void mouseWheel(MouseEvent event) override;
	void mouseUp(MouseEvent event) override;
	void resize() override;
	void update() override;
	void draw() override;
private:

    void readZMQData();
    std::atomic<bool> runReader_{true};
    std::mutex mtx_;
    std::thread zmqReader_;

	CameraPersp cam_;
	CameraUi cameraUi_{&cam_};
	TriMeshRef arrowMesh_;
	gl::GlslProgRef shader_;
	gl::BatchRef arrow_;
	
	std::vector<TriMeshRef> meshes_;
	std::vector<gl::BatchRef> batches_;

	gl::TextureRef textTexture_;
	Font font_{"Arial", 24};
	
	vec3 pos_{0.0, 1.0f, 0.0f};
	vec3 initialPos_{1.4f, 1.75f, 0.0f};
	quat rot_;
	vec3 initialRot_{0.0, 0.0f, 0.0f};

	vec3 gazeNormalized_{0.0f, 0.0f, -1.0f};
	zmq::context_t context_;
	zmq::socket_t sub_socket_;
	bool mouseDown_{false};
	bool useGaze_{true};
	bool drawFrame_{true};
	std::string focusedObject_{""};
};

void prepareSettings(GazeTracking::Settings *settings)
{
	settings->setMultiTouchEnabled(false);
	settings->setWindowSize(2400, 1600);
}

void GazeTracking::setup()
{
	std::ifstream ifstream("coco.ini");
	
	if (ifstream) {
		json j;
		ifstream >> j;

		auto initialPos = j["initialPos"];
		auto initialRot = j["initialRot"];

		initialPos_.x = initialPos[0];
		initialPos_.y = initialPos[1];
		initialPos_.z = initialPos[2];
		
		initialRot_.x = initialRot[0];
		initialRot_.y = initialRot[1];
		initialRot_.z = initialRot[2];
	}

	cam_.setPerspective(60, getWindowWidth() / static_cast<float>(getWindowHeight()), 1, 1000);
	cam_.lookAt({0.0f, 0.0f, 10.0f}, {0.0f, 0.0f, 0.0f});

	// auto lambert = gl::ShaderDef().lambert().color();
	// auto shader = gl::getStockShader(lambert);
	shader_ = gl::GlslProg::create( loadAsset( "shader.vert" ), loadAsset( "shader.frag" ) );

	arrowMesh_ = TriMesh::create(
		(geom::Cone().base(0.08f).apex(0.0f).height(0.1f) & (geom::Cylinder().radius(0.02f).height(0.2f) >> geom::Translate(0.0f, -0.2f, 0.0f))) >> geom::Rotate(-M_PI / 2.0f, {1.0f, 0.0f, 0.0f}));

	arrow_ = gl::Batch::create(*arrowMesh_, shader_);

	ObjLoader loader( loadAsset("PEEK_3dmodel.obj") );
	auto mesh = TriMesh::create( loader );
	
	meshes_.push_back(mesh);
	batches_.push_back(gl::Batch::create(*mesh, shader_));
	
	AxisAlignedBox bbox;

	loader = loadAsset("obj1.obj");
	auto source = TriMesh( loader );
	bbox = source.calcBoundingBox();
	source = source >> geom::Translate(-bbox.getCenter()) >> geom::Rotate(1.5f*M_PI, vec3(1.0f, 0.0f, 0.0f)) >> geom::Translate(0.3f, 2.3f, -0.3f);
	mesh = TriMesh::create(source);	
	meshes_.push_back(mesh);
	batches_.push_back(gl::Batch::create(*mesh, shader_));

	loader = loadAsset("obj2.obj");
	source = TriMesh( loader );
	bbox = source.calcBoundingBox();
	source = source >> geom::Translate(-bbox.getCenter()) >> geom::Rotate(1.5f*M_PI, vec3(1.0f, 0.0f, 0.0f)) >> geom::Translate(2.6f, 1.55f, -6.25f);
	mesh = TriMesh::create(source);
	meshes_.push_back(mesh);
	batches_.push_back(gl::Batch::create(*mesh, shader_));

	loader = loadAsset("obj3.obj");
	source = TriMesh( loader );
	bbox = source.calcBoundingBox();
	source = source >> geom::Translate(-bbox.getCenter()) >> geom::Rotate(1.5f*M_PI, vec3(1.0f, 0.0f, 0.0f)) >> geom::Translate(2.2f, 1.5f, -3.0f);
	mesh = TriMesh::create(source);
	meshes_.push_back(mesh);
	batches_.push_back(gl::Batch::create(*mesh, shader_));

	ImGui::Initialize();

	// // construct a REQ (request) socket and connect to interface
	// zmq::socket_t socket{context_, zmq::socket_type::req};
	// socket.set(zmq::sockopt::linger, 0);
	// socket.connect("tcp://localhost:50020");

	// // set up some static data to send
	// const std::string sub_request{"SUB_PORT"};
	// socket.send(zmq::buffer(sub_request), zmq::send_flags::dontwait);
	
	// std::this_thread::sleep_for(100ms);

	// bool pupil_detected = true;
	// zmq::message_t reply{};
	// auto res = socket.recv(reply, zmq::recv_flags::dontwait);

	// if (!res)
	// {
	// 	pupil_detected = false;
	// }

	// if (pupil_detected)
	// {
    //     std::cout << "Pupil detected" << std::endl;
	// 	std::string sub_port = reply.to_string();

	// 	const std::string pub_request{"PUB_PORT"};
	// 	socket.send(zmq::buffer(pub_request), zmq::send_flags::none);
	// 	// wait for reply from server
	// 	socket.recv(reply, zmq::recv_flags::none);
	// 	std::string pub_port = reply.to_string();

	// 	sub_socket_ = zmq::socket_t{context_, zmq::socket_type::sub};
	// 	sub_socket_.connect(std::string("tcp://localhost:") + sub_port);
		
    //     sub_socket_.set(zmq::sockopt::subscribe, "gaze.");
    //     sub_socket_.set(zmq::sockopt::subscribe, "tracking");
    // }
    
    // socket.close();

    sub_socket_ = zmq::socket_t{context_, zmq::socket_type::sub};
    sub_socket_.connect(std::string("tcp://macbook.local:9999"));

    zmqReader_ = std::thread(&GazeTracking::readZMQData, this);
}

GazeTracking::~GazeTracking() {
    runReader_ = false;
    zmqReader_.join();

	json j;
	j["initialPos"] = {initialPos_.x, initialPos_.y, initialPos_.z};
	j["initialRot"] = {initialRot_.x, initialRot_.y, initialRot_.z};
	std::ofstream ofstream("coco.ini");
	ofstream << j;
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
}

void GazeTracking::readZMQData() {
    while (runReader_) {
        if (sub_socket_)
        {
            std::vector<zmq::message_t> recv_msgs;
            zmq::recv_result_t result = zmq::recv_multipart(sub_socket_, std::back_inserter(recv_msgs));

            std::string topic((const char *)recv_msgs[0].data(), recv_msgs[0].size());

            if (topic == "position") {
                msgpack::object_handle oh = msgpack::unpack((const char *)recv_msgs[1].data(), recv_msgs[1].size());
                msgpack::object obj = oh.get();

                std::ostringstream oss;
                oss << obj;
                auto j = json::parse(oss.str());

                std::cout << j << std::endl;
            }
            else if (topic == "rotation") {

            }

        //     if (topic == "gaze.3d.0.") {

        //         msgpack::object_handle oh = msgpack::unpack((const char *)recv_msgs[1].data(), recv_msgs[1].size());
        //         msgpack::object obj = oh.get();

        //         std::ostringstream oss;
        //         oss << obj;
        //         auto j = json::parse(oss.str());

        //         auto confidence = j["confidence"];
        //         auto gaze_point_3d = j["gaze_normal_3d"];

        //         float c = static_cast<float>(confidence);

        //         if (c > 0.9f) {
        //             float x = static_cast<float>(gaze_point_3d[0]);
        //             float y = static_cast<float>(gaze_point_3d[1]);
        //             float z = static_cast<float>(gaze_point_3d[2]);

        //             if (useGaze_) {
        //                 std::lock_guard<std::mutex> lock(mtx_);
		// 				gazeNormalized_ = {x, -y, -z};
        //             } else {
        //                 std::lock_guard<std::mutex> lock(mtx_);
        //                 gazeNormalized_ = {0.0f, 0.0f, -1.0f};
        //             }
        //         }
        //     }
        //     else if (topic == "tracking") {
        //         msgpack::object_handle oh = msgpack::unpack((const char *)recv_msgs[1].data(), recv_msgs[1].size());
        //         msgpack::object obj = oh.get();

        //         std::ostringstream oss;
        //         oss << obj;
        //         try {
        //             auto j = json::parse(oss.str());

        //             auto motion = j["motion"];
        //             std::lock_guard<std::mutex> lock(mtx_);
        //             pos_ = vec3{motion[0], motion[1], motion[2]};
        //             rot_ = {motion[6], motion[3], motion[4], motion[5]};
        //         }
        //         catch (...) {
        //             std::cout << "error during json parsing" << std::endl;
        //         }
        //     }
        }

        std::this_thread::yield();
    }
}

void GazeTracking::draw()
{
	ImGui::Begin( "Info" );
	ImGui::Separator();
	ImGui::DragFloat3("Initial pos", &initialPos_[0], 0.2f, -10.0f, 10.0f);
	ImGui::DragFloat3("Initial rot", &initialRot_[0], 0.0f, -glm::pi<float>(), glm::pi<float>());
	ImGui::Separator();
	ImGui::Checkbox("Draw frame", &drawFrame_);
	ImGui::Separator();
	ImGui::Text("Focused object %s", focusedObject_.c_str());
	ImGui::End();

	shader_->uniform("uLightPos", vec3(100.0, 100.0, 100.0));

	{
		gl::ScopedMatrices scope;

		gl::clear(Color::gray(0.1f));
		gl::color(1.0f, 0.5f, 0.25f);
		gl::enableDepthRead();
		gl::enableDepthWrite();
		
		gl::enableFaceCulling(true);
		
		// gl::polygonMode(GL_FRONT_AND_BACK, GL_FILL);
		// gl::ScopedFaceCulling cull(true, GL_BACK);

		gl::setMatrices(cam_);

		gl::enableWireframe();

		vec3 gazeVec;
        vec3 pos;
        quat rot;

        {
            std::lock_guard<std::mutex> lock(mtx_);
			gazeVec = gazeNormalized_;
            pos = pos_ + initialPos_;
			auto initialRot = quat(initialRot_);
            rot = rot_*initialRot;
        }

		vec3 rotatedGazeVec = rot*gazeVec;

		quat eyeQuat = rotation({0.0f, 0.0f, -1.0f}, gazeVec);

		if (drawFrame_) {
			gl::drawCoordinateFrame(5.0f);
		}

		Ray ray{pos, rotatedGazeVec};

		TextBox tbox = TextBox().alignment(TextBox::LEFT).font(font_).size(ivec2(600, TextBox::GROW)).text("");
		tbox.setColor(Color(1.0f, 0.65f, 0.35f));
		tbox.setBackgroundColor(ColorA(0.5, 0, 0, 1));
		textTexture_ = gl::Texture2d::create(tbox.render());

		int id = 0;
		focusedObject_ = "-";

		for (int i = 0; i < meshes_.size(); ++i) {
			
			auto& mesh = meshes_[i];

			/*
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
			*/

			auto bbox = mesh->calcBoundingBox();
			bbox = AxisAlignedBox(bbox.getCenter()-3.0f*bbox.getExtents(), bbox.getCenter()+3.0f*bbox.getExtents());

			if (i == 0) {
				gl::disableWireframe();
				batches_[i]->draw();
				gl::enableWireframe();
			}

			// Did we have a hit?
			//else if( distance > 0.0f) {
			else if (bbox.intersects(ray) && dot(bbox.getCenter()-ray.getOrigin(), ray.getDirection()) > 0.0f) {
				gl::disableWireframe();
				gl::color( Color( CM_HSV, i/3.0, 1, 1 ) );
				batches_[i]->draw();
				gl::enableWireframe();

				// Calculate the exact position of the hit.
				// auto pickedPoint = ray.calcPosition( result );
				// gl::ScopedColor color(0.0f, 0.0f, 1.0f, 0.5f);
				// gl::drawCube(pickedPoint, {2.0f, 2.0f, 2.0f});

				// std::ostringstream oss_gp;
				// oss_gp << "Object " << id << std::endl;
				// TextBox tbox = TextBox().alignment(TextBox::LEFT).font(font_).size(ivec2(600, TextBox::GROW)).text(oss_gp.str());
				// tbox.setColor(Color(1.0f, 0.65f, 0.35f));
				// tbox.setBackgroundColor(ColorA(0.5, 0, 0, 1));
				// textTexture_ = gl::Texture2d::create(tbox.render());
				gl::drawStrokedCube(bbox);
				focusedObject_ = std::to_string(id);
			} else {
				gl::disableWireframe();
				gl::color( Color( CM_HSV, i/3.0, 1, 1 ) );
				batches_[i]->draw();
				gl::enableWireframe();
			}

			id++;

		}

		 gl::disableWireframe();

		// gl::drawLine(pos, pos + 100.0f*rotatedGazeVec);

		{
			gl::ScopedMatrices scope;
			gl::translate(pos.x, pos.y, pos.z);
			gl::rotate(-rot * eyeQuat);
			arrow_->draw();
		}
	}

	if (textTexture_)
		gl::draw(textTexture_);
}

CINDER_APP(GazeTracking, RendererGl, prepareSettings)
