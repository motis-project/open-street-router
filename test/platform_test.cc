#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/platforms.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, platform) {
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract(true, "test/ffm_hbf.osm", "/tmp/osr_test");

  auto w = osr::ways{"/tmp/osr_test", cista::mmap::protection::READ};
  auto pl = platforms{p, cista::mmap::protection::READ};

  for (auto i = platform_idx_t{0U}; i != pl.platform_ref_.size(); ++i) {
    fmt::println("platform {}", i);
    for (auto j = 0U; j != pl.platform_names_[i].size(); ++j) {
      fmt::println("  name: {}", pl.platform_names_[i][j].view());
    }
    for (auto j = 0U; j != pl.platform_ref_[i].size(); ++j) {
      std::visit(utl::overloaded{[&](node_idx_t const n) {
                                   fmt::println("  node {}", w.node_to_osm_[n]);
                                 },
                                 [&](way_idx_t const way) {
                                   fmt::println("  way {}",
                                                w.way_osm_idx_[way]);
                                 }},
                 to_ref(pl.platform_ref_[i][j]));
    }
  }
}
