#pragma once

#include <memory>
#include <string>

#include "boost/asio/io_context.hpp"

#include "osr/lookup.h"
#include "osr/platforms.h"
#include "osr/ways.h"

namespace osr::backend {

struct http_server {
  http_server(boost::asio::io_context& ioc,
              boost::asio::io_context& thread_pool,
              ways const&,
              platforms const*,
              lookup const&,
              std::string const& static_file_path);
  ~http_server();
  http_server(http_server const&) = delete;
  http_server& operator=(http_server const&) = delete;
  http_server(http_server&&) = delete;
  http_server& operator=(http_server&&) = delete;

  void listen(std::string const& host, std::string const& port);
  void stop();

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::backend
