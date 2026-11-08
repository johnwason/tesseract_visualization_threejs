#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <tesseract/common/logging.h>
#include <tesseract/visualization/threejs/viewer_server.h>
#include <tesseract/visualization/threejs/viewer_state.h>

namespace tesseract::visualization::threejs
{
namespace
{
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// A client that stops reading is dropped rather than allowed to queue messages without limit
constexpr std::size_t MAX_QUEUED_MESSAGES = 256;
constexpr auto HTTP_TIMEOUT = std::chrono::seconds(30);
constexpr std::size_t MAX_INCOMING_WEBSOCKET_MESSAGE = 64 * 1024;

class WebsocketSession;

/** @brief Open websockets. The set is only changed and iterated on the server thread */
class Hub
{
public:
  void add(const std::shared_ptr<WebsocketSession>& session)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sessions_.insert(session);
    connected_.notify_all();
  }

  void remove(const std::shared_ptr<WebsocketSession>& session)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sessions_.erase(session);
  }

  std::size_t count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return sessions_.size();
  }

  bool waitForConnection(std::chrono::milliseconds timeout) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return connected_.wait_for(lock, timeout, [this] { return !sessions_.empty(); });
  }

  std::set<std::shared_ptr<WebsocketSession>> snapshot() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return sessions_;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable connected_;
  std::set<std::shared_ptr<WebsocketSession>> sessions_;
};

class WebsocketSession : public std::enable_shared_from_this<WebsocketSession>
{
public:
  WebsocketSession(tcp::socket&& socket, std::shared_ptr<Hub> hub) : ws_(std::move(socket)), hub_(std::move(hub)) {}

  void run(http::request<http::string_body> request)
  {
    beast::get_lowest_layer(ws_).expires_never();
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.read_message_max(MAX_INCOMING_WEBSOCKET_MESSAGE);
    request_ = std::move(request);
    ws_.async_accept(request_, [self = shared_from_this()](beast::error_code ec) {
      if (ec)
        return;
      self->hub_->add(self);
      self->read();
    });
  }

  void send(const std::shared_ptr<const std::string>& message)
  {
    if (queue_.size() >= MAX_QUEUED_MESSAGES)
    {
      TESSERACT_LOG_WARN("tesseract_visualization_threejs: dropping a websocket client that is not reading");
      close();
      return;
    }
    queue_.push_back(message);
    if (queue_.size() == 1)
      write();
  }

private:
  // Display only: incoming messages are read so that pings and closes are handled, then discarded
  void read()
  {
    ws_.async_read(buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t) {
      if (ec)
      {
        self->hub_->remove(self);
        return;
      }
      self->buffer_.consume(self->buffer_.size());
      self->read();
    });
  }

  void write()
  {
    ws_.text(true);
    ws_.async_write(net::buffer(*queue_.front()), [self = shared_from_this()](beast::error_code ec, std::size_t) {
      if (ec)
      {
        self->hub_->remove(self);
        return;
      }
      self->queue_.pop_front();
      if (!self->queue_.empty())
        self->write();
    });
  }

  void close()
  {
    hub_->remove(shared_from_this());
    beast::error_code ignored;
    beast::get_lowest_layer(ws_).socket().close(ignored);
  }

  websocket::stream<beast::tcp_stream> ws_;
  std::shared_ptr<Hub> hub_;
  http::request<http::string_body> request_;
  beast::flat_buffer buffer_;
  std::deque<std::shared_ptr<const std::string>> queue_;
};

class HttpSession : public std::enable_shared_from_this<HttpSession>
{
public:
  HttpSession(tcp::socket&& socket, std::shared_ptr<ViewerState> state, std::shared_ptr<Hub> hub)
    : stream_(std::move(socket)), state_(std::move(state)), hub_(std::move(hub))
  {
  }

  void read()
  {
    request_ = {};
    stream_.expires_after(HTTP_TIMEOUT);
    http::async_read(stream_, buffer_, request_, [self = shared_from_this()](beast::error_code ec, std::size_t) {
      self->onRead(ec);
    });
  }

private:
  void onRead(beast::error_code ec)
  {
    if (ec == http::error::end_of_stream)
    {
      stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
      return;
    }
    if (ec)
      return;

    if (websocket::is_upgrade(request_) && targetPath() == "/websocket")
    {
      std::make_shared<WebsocketSession>(stream_.release_socket(), hub_)->run(std::move(request_));
      return;
    }

    respond();
  }

  std::string targetPath() const
  {
    std::string target(request_.target());
    return target.substr(0, target.find('?'));
  }

  void respond()
  {
    auto response = std::make_shared<http::response<http::string_body>>(http::status::ok, request_.version());
    response->set(http::field::server, "tesseract_visualization_threejs");
    response->keep_alive(request_.keep_alive());

    const bool is_head = request_.method() == http::verb::head;
    if (request_.method() != http::verb::get && !is_head)
    {
      response->result(http::status::method_not_allowed);
      response->set(http::field::allow, "GET, HEAD");
      send(response, std::make_shared<const std::string>(), false);
      return;
    }

    const std::string path = targetPath();
    if (path == "/" || path == "/index.html")
      sendStatic(response, "index.html", "text/html; charset=utf-8", is_head);
    else if (path == "/app.js")
      sendStatic(response, "app.js", "text/javascript; charset=utf-8", is_head);
    else if (path == "/tesseract_scene.gltf")
      sendScene(response, state_->sceneGltf(), "model/gltf+json", "{}", is_head);
    else if (path == "/tesseract_scene.glb")
      sendScene(response, state_->sceneGlb(), "application/octet-stream", "", is_head);
    else if (path == "/tesseract_trajectory.json")
      sendDocument(response, state_->trajectoryJson(), is_head);
    else if (path == "/tesseract_markers.json")
      sendDocument(response, state_->markersJson(), is_head);
    else
    {
      response->result(http::status::not_found);
      send(response, std::make_shared<const std::string>(), is_head);
    }
  }

  using Response = std::shared_ptr<http::response<http::string_body>>;

  void sendStatic(const Response& response, const char* name, const char* content_type, bool is_head)
  {
    response->set(http::field::content_type, content_type);
    send(response, std::make_shared<const std::string>(staticFile(name)), is_head);
  }

  void sendScene(const Response& response,
                 const ViewerResource& resource,
                 const char* content_type,
                 const char* empty_body,
                 bool is_head)
  {
    response->set(http::field::content_type, content_type);
    response->set(http::field::etag, "\"" + resource.etag + "\"");
    send(response, resource.body != nullptr ? resource.body : std::make_shared<const std::string>(empty_body), is_head);
  }

  void sendDocument(const Response& response, const ViewerResource& resource, bool is_head)
  {
    if (resource.body == nullptr)
    {
      response->result(http::status::not_found);
      send(response, std::make_shared<const std::string>(), is_head);
      return;
    }
    response->set(http::field::content_type, "application/json");
    response->set(http::field::etag, "\"" + resource.etag + "\"");
    send(response, resource.body, is_head);
  }

  void send(const Response& response, const std::shared_ptr<const std::string>& body, bool is_head)
  {
    response->content_length(body->size());
    if (!is_head)
      response->body() = *body;

    stream_.expires_after(HTTP_TIMEOUT);
    http::async_write(stream_, *response, [self = shared_from_this(), response](beast::error_code ec, std::size_t) {
      if (ec)
        return;
      if (!response->keep_alive())
      {
        self->stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
        return;
      }
      self->read();
    });
  }

  beast::tcp_stream stream_;
  std::shared_ptr<ViewerState> state_;
  std::shared_ptr<Hub> hub_;
  beast::flat_buffer buffer_;
  http::request<http::string_body> request_;
};
}  // namespace

class ViewerServer::Impl
{
public:
  Impl(std::shared_ptr<ViewerState> state, std::string host, unsigned short port)
    : state_(std::move(state)), host_(std::move(host)), port_(port), hub_(std::make_shared<Hub>())
  {
  }

  ~Impl() { stop(); }

  void start()
  {
    const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (thread_.joinable())
      return;

    ioc_.restart();
    tcp::resolver resolver(ioc_);
    const tcp::endpoint endpoint = *resolver.resolve(host_, std::to_string(port_), tcp::resolver::passive).begin();

    acceptor_ = std::make_unique<tcp::acceptor>(ioc_);
    acceptor_->open(endpoint.protocol());
#ifndef _WIN32
    // On Windows this option would let a second server bind a port that is already in use
    acceptor_->set_option(net::socket_base::reuse_address(true));
#endif
    acceptor_->bind(endpoint);
    acceptor_->listen(net::socket_base::max_listen_connections);
    bound_port_ = acceptor_->local_endpoint().port();

    state_->setMessageCallback([this, hub = hub_](std::shared_ptr<const std::string> message) {
      net::post(ioc_, [hub, message = std::move(message)] {
        for (const auto& session : hub->snapshot())
          session->send(message);
      });
    });

    accept();
    thread_ = std::thread([this] { ioc_.run(); });
  }

  void stop()
  {
    const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!thread_.joinable())
      return;
    state_->setMessageCallback({});
    ioc_.stop();
    thread_.join();
    acceptor_.reset();
    for (const auto& session : hub_->snapshot())
      hub_->remove(session);
  }

  unsigned short port() const { return bound_port_; }
  const std::shared_ptr<Hub>& hub() const { return hub_; }

private:
  void accept()
  {
    acceptor_->async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (ec == net::error::operation_aborted)
        return;
      if (!ec)
        std::make_shared<HttpSession>(std::move(socket), state_, hub_)->read();
      accept();
    });
  }

  std::shared_ptr<ViewerState> state_;
  std::string host_;
  unsigned short port_;
  std::shared_ptr<Hub> hub_;
  std::mutex lifecycle_mutex_;
  net::io_context ioc_{ 1 };
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread thread_;
  unsigned short bound_port_{ 0 };
};

ViewerServer::ViewerServer(std::shared_ptr<ViewerState> state, std::string host, unsigned short port)
  : impl_(std::make_unique<Impl>(std::move(state), std::move(host), port))
{
}

ViewerServer::~ViewerServer() = default;

void ViewerServer::start() { impl_->start(); }
void ViewerServer::stop() { impl_->stop(); }
unsigned short ViewerServer::port() const { return impl_->port(); }
std::size_t ViewerServer::connectionCount() const { return impl_->hub()->count(); }
bool ViewerServer::waitForConnection(std::chrono::milliseconds timeout) const
{
  return impl_->hub()->waitForConnection(timeout);
}

}  // namespace tesseract::visualization::threejs
