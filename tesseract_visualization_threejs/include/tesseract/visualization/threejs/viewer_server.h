/**
 * @file viewer_server.h
 * @brief Layer 3: serves a ViewerState and the client over HTTP and websocket with Boost.Beast
 */
#ifndef TESSERACT_VISUALIZATION_THREEJS_VIEWER_SERVER_H
#define TESSERACT_VISUALIZATION_THREEJS_VIEWER_SERVER_H

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

namespace tesseract::visualization::threejs
{
class ViewerState;

/** @brief Display only: messages received from clients are discarded */
class ViewerServer
{
public:
  /**
   * @param state The state to serve
   * @param host Address to bind. Localhost by default because the viewer has no authentication
   * @param port Port to bind. 0 picks a free port; see port()
   */
  explicit ViewerServer(std::shared_ptr<ViewerState> state, std::string host = "127.0.0.1", unsigned short port = 8000);
  ~ViewerServer();
  ViewerServer(const ViewerServer&) = delete;
  ViewerServer& operator=(const ViewerServer&) = delete;

  /** @brief Bind and serve on a background thread. @throws std::exception if the address cannot be bound */
  void start();
  void stop();

  /** @brief The bound port, valid after start() */
  unsigned short port() const;

  /** @brief The number of open websocket connections */
  std::size_t connectionCount() const;

  /** @brief Wait for a websocket connection. @return true if connected before the timeout */
  bool waitForConnection(std::chrono::milliseconds timeout) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tesseract::visualization::threejs

#endif  // TESSERACT_VISUALIZATION_THREEJS_VIEWER_SERVER_H
