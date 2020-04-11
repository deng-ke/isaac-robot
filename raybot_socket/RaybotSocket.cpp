#include "RaybotSocket.hpp"

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/backend/asio_backend.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/logger.hpp"

namespace {
  std::array<char, 20> data;
  //std::shared_ptr<std::string> buf;
}
namespace isaac {

  RaybotSocket::RaybotSocket() {}

  RaybotSocket::~RaybotSocket() {}

  void RaybotSocket::acceptConnection() {
    ASSERT(acceptor_, "Acceptor not initialized");
    pending_socket_ = std::make_shared<asio::ip::tcp::socket>(backend_->io_service());
    ASSERT(pending_socket_, "Could not create socket");
    acceptor_->async_accept(*pending_socket_,
      [this](std::error_code ec) {
        std::cout << ec << std::endl;
        if (ec) {
          if (ec == asio::error::operation_aborted) {
            LOG_ERROR("aborted");
            return;
          }
          LOG_ERROR("async_accept failed: %s", ec.message().c_str());
        } else {
          LOG_INFO("TCP transmitter successfully accepted conntection");
          sockets_.emplace_back(std::move(pending_socket_));
          pending_socket_ = nullptr;
        }
        writeMessage();
        acceptConnection();
      });
  }

  void RaybotSocket::writeMessage() {
    std::error_code ec;
    time_t now = time(0);
    std::shared_ptr<std::string> message(new std::string(ctime(&now)));
    for(auto& socket : sockets_) {
      asio::ip::tcp::socket* socket_ptr = socket.get();
      socket_ptr->async_write_some(asio::buffer(*message), 
          std::bind(&RaybotSocket::writeHandler, this, std::placeholders::_1));
    }
  }

  void RaybotSocket::readMessage() {
    std::error_code ec;
    size_t len;
    for(auto& socket : sockets_) {
      asio::ip::tcp::socket* socket_ptr = socket.get();
      socket_ptr->async_read_some(asio::buffer(data), 
          std::bind(&RaybotSocket::readHandler, this, std::placeholders::_1));
    }
  }

  void RaybotSocket::writeHandler(const std::error_code& ec ) {
    if(ec) {
      LOG_WARNING("Could not write to socket!");
    }
    std::cout << "write completed" << std::endl;
  }

  void RaybotSocket::readHandler(const std::error_code& ec) {
    if(ec) {
      LOG_WARNING("Could not read form socket!");
    }
    std::cout << "data: " << data.data() << std::endl;
  }

  void RaybotSocket::start() {
    std::cout << "server start." << std::endl;
    backend_ = node()->app()->backend()->asio_backend();
    acceptor_ = std::make_shared<asio::ip::tcp::acceptor>(
      backend_->io_service(), asio::ip::tcp::endpoint(asio::ip::tcp::v4(), get_port()));
    acceptConnection();
    tickPeriodically();
  }
#if 1
  void RaybotSocket::tick() {
    sleep(1);
    writeMessage();
    readMessage();
  }
#endif
  void RaybotSocket::stop() {
    if(acceptor_) {
      acceptor_->cancel();
      acceptor_.reset();
    }
    pending_socket_.reset();
    sockets_.clear();
    backend_ = nullptr;
    std::cout << "server closed" << std::endl;
  }
}  // namespace isaac
