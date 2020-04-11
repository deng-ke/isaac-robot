#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <iostream>
#include <functional>

#include "asio.hpp"  // NOLINT(build/include)
#include "engine/alice/component.hpp"
#include "engine/alice/message.hpp"
#include "engine/alice/alice_codelet.hpp"

namespace isaac {
  namespace alice {
    class AsioBackend;
  }
}
namespace isaac {

  class RaybotSocket: public alice::Codelet {
    public:
      RaybotSocket();
      ~RaybotSocket();
      void start() override;
      void tick() override;
      void stop() override;
      void acceptConnection();
      void writeMessage();
      void readMessage();
      void writeHandler(const std::error_code& ec);
      void readHandler(const std::error_code& ec);

      // The TCP port number
      ISAAC_PARAM(int, port);

    private:
      std::shared_ptr<asio::ip::tcp::acceptor> acceptor_;
      std::shared_ptr<asio::ip::tcp::socket> pending_socket_;
      std::vector<std::shared_ptr<asio::ip::tcp::socket>> sockets_;
      asio::io_service io_service_;
      alice::AsioBackend* backend_;
  };

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::RaybotSocket)
