#include <gtest/gtest.h>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <cstdlib>
#include <thread>

#include <json/json.h>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/viewer/viewer_server.h>
#include <tesseract/viewer/viewer_state.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/visualization_loader.h>

using namespace tesseract::viewer;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using Response = http::response<http::string_body>;

namespace
{
constexpr char URDF[] = R"(
<robot name="r" tesseract:make_convex="true" xmlns:tesseract="http://tesseract">
  <link name="base"/>
  <link name="l1"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit effort="0" lower="-2" upper="2" velocity="6"/>
  </joint>
</robot>)";

Response request(unsigned short port, http::verb verb, const std::string& target)
{
  net::io_context ioc;
  tcp::socket socket(ioc);
  socket.connect({ net::ip::make_address("127.0.0.1"), port });
  http::request<http::empty_body> req(verb, target, 11);
  req.set(http::field::host, "localhost");
  req.keep_alive(false);
  http::write(socket, req);

  beast::flat_buffer buffer;
  http::response_parser<http::string_body> parser;
  parser.body_limit(100 * 1024 * 1024);
  parser.skip(verb == http::verb::head);
  http::read(socket, buffer, parser);
  return parser.release();
}

Response get(unsigned short port, const std::string& target) { return request(port, http::verb::get, target); }
Response head(unsigned short port, const std::string& target) { return request(port, http::verb::head, target); }

std::string etagOf(const Response& response) { return std::string(response[http::field::etag]); }

class WebsocketClient
{
public:
  explicit WebsocketClient(unsigned short port) : ws_(ioc_)
  {
    beast::get_lowest_layer(ws_).connect({ net::ip::make_address("127.0.0.1"), port });
    ws_.handshake("localhost", "/websocket");
  }

  Json::Value read()
  {
    beast::flat_buffer buffer;
    ws_.read(buffer);
    Json::Value out;
    const std::string text = beast::buffers_to_string(buffer.data());
    std::string errors;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    EXPECT_TRUE(reader->parse(text.data(), text.data() + text.size(), &out, &errors)) << text;
    return out;
  }

private:
  net::io_context ioc_;
  websocket::stream<beast::tcp_stream> ws_;
};

class ServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    state = std::make_shared<ViewerState>();
    server = std::make_unique<ViewerServer>(state, "127.0.0.1", 0);
    server->start();
    port = server->port();
    ASSERT_NE(port, 0);
  }

  std::shared_ptr<ViewerState> state;
  std::unique_ptr<ViewerServer> server;
  unsigned short port{ 0 };
};
}  // namespace

TEST_F(ServerTest, ServesTheClient)
{
  for (const char* target : { "/", "/index.html" })
  {
    const Response index = get(port, target);
    EXPECT_EQ(index.result(), http::status::ok);
    EXPECT_EQ(index.body(), std::string(staticFile("index.html")));
    EXPECT_NE(index[http::field::content_type].find("text/html"), beast::string_view::npos);
  }

  const Response app = get(port, "/app.js");
  EXPECT_EQ(app.result(), http::status::ok);
  EXPECT_EQ(app.body(), std::string(staticFile("app.js")));
  // Browsers refuse module scripts without a JavaScript mime type
  EXPECT_NE(app[http::field::content_type].find("javascript"), beast::string_view::npos);

  const Response head_app = head(port, "/app.js");
  EXPECT_EQ(head_app.result(), http::status::ok);
  EXPECT_EQ(head_app.body(), "");
  EXPECT_EQ(head_app[http::field::content_length], std::to_string(staticFile("app.js").size()));
}

TEST_F(ServerTest, BeforeAnythingIsSet)
{
  const Response gltf = get(port, "/tesseract_scene.gltf");
  EXPECT_EQ(gltf.result(), http::status::ok);
  EXPECT_EQ(gltf.body(), "{}");
  EXPECT_EQ(etagOf(gltf), "\"None\"");

  const Response glb = get(port, "/tesseract_scene.glb");
  EXPECT_EQ(glb.result(), http::status::ok);
  EXPECT_EQ(glb.body(), "");
  EXPECT_EQ(etagOf(head(port, "/tesseract_scene.glb")), "\"None\"");

  EXPECT_EQ(get(port, "/tesseract_trajectory.json").result(), http::status::not_found);
  EXPECT_EQ(head(port, "/tesseract_trajectory.json").result(), http::status::not_found);
  EXPECT_EQ(get(port, "/tesseract_markers.json").result(), http::status::not_found);
}

TEST_F(ServerTest, DocumentsAndETags)
{
  state->setScene("{\"s\":1}", "GLB!");
  state->setTrajectoryJson("{\"t\":1}");
  state->addSphereMarker(Eigen::Vector3d::Zero(), 0.1);

  const Response gltf = get(port, "/tesseract_scene.gltf");
  EXPECT_EQ(gltf.body(), "{\"s\":1}");
  EXPECT_EQ(gltf[http::field::content_type], "model/gltf+json");
  EXPECT_EQ(etagOf(gltf), "\"" + hashBytes("{\"s\":1}") + "\"");
  EXPECT_EQ(etagOf(head(port, "/tesseract_scene.gltf")), etagOf(gltf));

  const Response glb = get(port, "/tesseract_scene.glb");
  EXPECT_EQ(glb.body(), "GLB!");
  EXPECT_EQ(glb[http::field::content_type], "application/octet-stream");

  const Response trajectory = get(port, "/tesseract_trajectory.json");
  EXPECT_EQ(trajectory.body(), "{\"t\":1}");
  EXPECT_EQ(trajectory[http::field::content_type], "application/json");

  // The Python viewer answered HEAD with a 500 here once a trajectory was set
  const Response trajectory_head = head(port, "/tesseract_trajectory.json");
  EXPECT_EQ(trajectory_head.result(), http::status::ok);
  EXPECT_EQ(etagOf(trajectory_head), etagOf(trajectory));
  EXPECT_EQ(trajectory_head[http::field::content_length], "7");

  const Response markers = get(port, "/tesseract_markers.json");
  EXPECT_EQ(markers.result(), http::status::ok);
  EXPECT_EQ(etagOf(head(port, "/tesseract_markers.json")), etagOf(markers));
}

TEST_F(ServerTest, RejectsWhatItDoesNotServe)
{
  EXPECT_EQ(get(port, "/nothing").result(), http::status::not_found);
  EXPECT_EQ(get(port, "/../CMakeLists.txt").result(), http::status::not_found);
  EXPECT_EQ(request(port, http::verb::post, "/tesseract_scene.gltf").result(), http::status::method_not_allowed);
}

TEST_F(ServerTest, QueryStringsAreIgnored)
{
  EXPECT_EQ(get(port, "/app.js?v=1").result(), http::status::ok);
}

TEST_F(ServerTest, KeepAliveServesSeveralRequests)
{
  net::io_context ioc;
  tcp::socket socket(ioc);
  socket.connect({ net::ip::make_address("127.0.0.1"), port });
  beast::flat_buffer buffer;
  for (int i = 0; i < 3; ++i)
  {
    http::request<http::empty_body> req(http::verb::get, "/index.html", 11);
    req.set(http::field::host, "localhost");
    http::write(socket, req);
    Response response;
    http::read(socket, buffer, response);
    EXPECT_EQ(response.result(), http::status::ok);
  }
}

TEST_F(ServerTest, WebsocketReceivesCommandsInOrder)
{
  WebsocketClient first(port);
  WebsocketClient second(port);
  ASSERT_TRUE(server->waitForConnection(std::chrono::seconds(5)));
  for (int i = 0; i < 500 && server->connectionCount() < 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(server->connectionCount(), 2u);

  state->setScene("{}", "x");
  state->setJointPositions({ "j" }, Eigen::VectorXd::Constant(1, 0.25));
  state->addAxesMarker(Eigen::Isometry3d::Identity());

  for (WebsocketClient* client : { &first, &second })
  {
    EXPECT_EQ(client->read()["command"].asString(), "refresh_scene");
    const Json::Value trajectory = client->read();
    EXPECT_EQ(trajectory["command"].asString(), "joint_trajectory");
    EXPECT_EQ(trajectory["params"]["joint_names"][0].asString(), "j");
    const Json::Value markers = client->read();
    EXPECT_EQ(markers["command"].asString(), "markers");
    EXPECT_EQ(markers["params"]["markers"][0]["marker_type"].asString(), "axes");
  }
}

TEST_F(ServerTest, ClosedWebsocketIsForgotten)
{
  {
    WebsocketClient client(port);
    ASSERT_TRUE(server->waitForConnection(std::chrono::seconds(5)));
  }
  for (int i = 0; i < 500 && server->connectionCount() > 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(server->connectionCount(), 0u);
}

TEST(Server, StopsAndRestarts)
{
  auto state = std::make_shared<ViewerState>();
  ViewerServer server(state, "127.0.0.1", 0);
  server.start();
  EXPECT_EQ(get(server.port(), "/").result(), http::status::ok);
  server.stop();
  state->setScene("{}", "");  // no callback may be left pointing at the stopped server
  server.start();
  EXPECT_EQ(get(server.port(), "/").result(), http::status::ok);
}

TEST(Server, PortInUseThrowsFromStart)
{
  auto state = std::make_shared<ViewerState>();
  ViewerServer first(state, "127.0.0.1", 0);
  first.start();
  ViewerServer second(std::make_shared<ViewerState>(), "127.0.0.1", first.port());
  EXPECT_ANY_THROW(second.start());
}

TEST(Plugin, LoadsAndServes)
{
  // Nothing else in this process uses the plugin's server, so a fixed port is safe within the test
  const unsigned short plugin_port = 18765;
  setenv("TESSERACT_VIEWER_PORT", std::to_string(plugin_port).c_str(), 1);

  tesseract::visualization::VisualizationLoader loader;
  auto named = loader.get("TesseractViewerVisualizationPlugin");
  ASSERT_NE(named, nullptr);
  // get() without a name asks for the Ignition symbol, which is how existing examples call it
  auto by_default = loader.get();
  ASSERT_NE(by_default, nullptr);

  EXPECT_FALSE(named->isConnected());
  WebsocketClient client(plugin_port);
  named->waitForConnection(5);
  EXPECT_TRUE(named->isConnected());
  EXPECT_TRUE(by_default->isConnected()) << "Both symbols must share one server";

  tesseract::environment::Environment env;
  ASSERT_TRUE(env.init(std::string(URDF), std::make_shared<tesseract::common::GeneralResourceLocator>()));

  named->plotEnvironment(env);
  EXPECT_EQ(client.read()["command"].asString(), "refresh_scene");
  EXPECT_EQ(get(plugin_port, "/tesseract_scene.glb").result(), http::status::ok);

  named->plotEnvironmentState(env.getState());
  Json::Value message = client.read();
  EXPECT_EQ(message["command"].asString(), "joint_trajectory");
  EXPECT_EQ(message["params"]["joint_names"][0].asString(), "j1");

  tesseract::visualization::AxisMarker axis(Eigen::Isometry3d::Identity());
  named->plotMarker(axis, "my_ns");
  message = client.read();
  EXPECT_EQ(message["command"].asString(), "markers");
  const Json::Value& marker = message["params"]["markers"][0];
  EXPECT_EQ(marker["marker_type"].asString(), "axes");
  EXPECT_EQ(marker["parent_link_name"].asString(), "base") << "A marker with no parent attaches to the root link";
  EXPECT_EQ(marker["tags"][0].asString(), "my_ns");

  named->clear("my_ns");
  message = client.read();
  EXPECT_EQ(message["params"]["markers"].size(), 0u);

  tesseract::common::JointTrajectory bad;
  bad.states.emplace_back(std::vector<tesseract::common::JointId>{ tesseract::common::JointId("j1") },
                          Eigen::VectorXd::Zero(2));
  EXPECT_NO_THROW(named->plotTrajectory(bad, *env.getStateSolver())) << "Plotting must not abort planning";
}
