#pragma once

#include <string_view>
#include <vector>

#include "fmt/ranges.h"

#include "utl/concat.h"
#include "utl/power_set_intersection.h"
#include "utl/to_vec.h"

#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/types.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kCar,
};

search_profile to_profile(std::string_view);

std::string_view to_str(search_profile);

struct path {
  struct segment {
    std::vector<geo::latlng> polyline_;
    level_t from_level_;
    level_t to_level_;
    way_idx_t way_;
  };

  cost_t cost_{kInfeasible};
  double dist_{0.0};
  std::vector<segment> segments_{};
};

struct connecting_way {
  way_idx_t way_;
  std::uint16_t from_, to_;
  bool is_loop_;
  std::uint16_t distance_;
};

template <typename Profile>
dijkstra<Profile>& get_dijkstra();

std::vector<std::optional<path>> route(
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr);

std::optional<path> route(ways const&,
                          lookup const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr);

}  // namespace osr