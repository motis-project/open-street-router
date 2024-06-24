#pragma once

#include "osr/ways.h"

#include "utl/cflow.h"
#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"

#include "rtree.h"

namespace osr {

struct location {
  geo::latlng pos_;
  level_t lvl_;
};

struct node_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  level_t lvl_{level_t::invalid()};
  direction way_dir_{direction::kForward};
  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
  cost_t cost_{0U};
  cost_t offroad_cost_{0U};
  std::vector<geo::latlng> path_{};
};

struct way_candidate {
  friend bool operator<(way_candidate const& a, way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  double dist_to_way_;
  geo::latlng best_;
  std::size_t segment_idx_;
  way_idx_t way_{way_idx_t::invalid()};
  node_candidate left_{}, right_{};
};

template <typename T, typename Collection, typename Fn>
void till_the_end(T const& start,
                  Collection const& c,
                  direction const dir,
                  Fn&& fn) {
  if (dir == direction::kForward) {
    for (auto i = start; i != c.size(); ++i) {
      if (fn(c[i]) == utl::cflow::kBreak) {
        break;
      }
    }
  } else {
    for (auto j = 0U; j <= start; ++j) {
      auto i = start - j;
      if (fn(c[i]) == utl::cflow::kBreak) {
        break;
      }
    }
  }
}

using match_t = std::vector<way_candidate>;

template <typename PolyLine>
way_candidate distance_to_way(geo::latlng const& x, PolyLine&& c) {
  auto min = std::numeric_limits<double>::max();
  auto best = geo::latlng{};
  auto best_segment_idx = 0U;
  auto segment_idx = 0U;
  for (auto const [a, b] : utl::pairwise(c)) {
    auto const candidate = geo::closest_on_segment(x, a, b);
    auto const dist = geo::distance(x, candidate);
    if (dist < min) {
      min = dist;
      best = candidate;
      best_segment_idx = segment_idx;
    }
    ++segment_idx;
  }
  return {.dist_to_way_ = min, .best_ = best, .segment_idx_ = best_segment_idx};
}

struct lookup {
  lookup(ways const& ways) : rtree_{rtree_new()}, ways_{ways} {
    utl::verify(rtree_ != nullptr, "rtree creation failed");
    for (auto i = way_idx_t{0U}; i != ways.n_ways(); ++i) {
      insert(i);
    }
  }

  ~lookup() { rtree_free(rtree_); }

  template <typename Profile>
  match_t get_way_candidates(location const& query,
                             bool const reverse,
                             direction const search_dir,
                             double const max_match_distance,
                             bitvec<node_idx_t> const* blocked) const {
    auto way_candidates = std::vector<way_candidate>{};
    find(query.pos_, [&](way_idx_t const way) {
      auto d = distance_to_way(query.pos_, ways_.way_polylines_[way]);
      if (d.dist_to_way_ < max_match_distance) {
        auto& wc = way_candidates.emplace_back(std::move(d));
        wc.way_ = way;
        wc.left_ =
            find_next_node<Profile>(wc, query, direction::kBackward, query.lvl_,
                                    reverse, search_dir, blocked);
        wc.right_ =
            find_next_node<Profile>(wc, query, direction::kForward, query.lvl_,
                                    reverse, search_dir, blocked);
      }
    });
    utl::sort(way_candidates);
    return way_candidates;
  }

  template <typename Profile>
  match_t match(location const& query,
                bool const reverse,
                direction const search_dir,
                double const max_match_distance,
                bitvec<node_idx_t> const* blocked) const {
    auto way_candidates = get_way_candidates<Profile>(
        query, reverse, search_dir, max_match_distance, blocked);
    if (way_candidates.empty()) {
      way_candidates = get_way_candidates<Profile>(
          query, reverse, search_dir, max_match_distance * 2U, blocked);
    }
    return way_candidates;
  }

  template <typename Profile>
  node_candidate find_next_node(way_candidate const& wc,
                                location const& query,
                                direction const dir,
                                level_t const lvl,
                                bool const reverse,
                                direction const search_dir,
                                bitvec<node_idx_t> const* blocked) const {
    auto const way_prop = ways_.r_->way_properties_[wc.way_];
    auto const edge_dir = reverse ? opposite(dir) : dir;
    auto const way_cost =
        Profile::way_cost(way_prop, flip(search_dir, edge_dir), 0U);
    if (way_cost == kInfeasible) {
      return node_candidate{};
    }

    auto const offroad_cost =
        Profile::way_cost(way_prop, flip(search_dir, edge_dir),
                          static_cast<distance_t>(wc.dist_to_way_));
    auto c = node_candidate{.lvl_ = lvl,
                            .way_dir_ = dir,
                            .dist_to_node_ = wc.dist_to_way_,
                            .cost_ = offroad_cost,
                            .offroad_cost_ = offroad_cost,
                            .path_ = {query.pos_, wc.best_}};
    auto const polyline = ways_.way_polylines_[wc.way_];
    auto const osm_nodes = ways_.way_osm_nodes_[wc.way_];

    till_the_end(wc.segment_idx_ + (dir == direction::kForward ? 1U : 0U),
                 utl::zip(polyline, osm_nodes), dir, [&](auto&& x) {
                   auto const& [pos, osm_node_idx] = x;

                   auto const segment_dist = geo::distance(c.path_.back(), pos);
                   c.dist_to_node_ += segment_dist;
                   c.cost_ +=
                       Profile::way_cost(way_prop, flip(search_dir, edge_dir),
                                         static_cast<distance_t>(segment_dist));
                   c.path_.push_back(pos);

                   auto const way_node = ways_.find_node_idx(osm_node_idx);
                   if (way_node.has_value() &&
                       (blocked == nullptr || !blocked->test(*way_node))) {
                     c.node_ = *way_node;
                     return utl::cflow::kBreak;
                   }

                   return utl::cflow::kContinue;
                 });

    if (reverse) {
      std::reverse(begin(c.path_), end(c.path_));
    }

    return c;
  }

  template <typename Fn>
  void find(geo::latlng const& x, Fn&& fn) const {
    find({x.lat() - 0.01, x.lng() - 0.01}, {x.lat() + 0.01, x.lng() + 0.01},
         std::forward<Fn>(fn));
  }

  template <typename Fn>
  void find(geo::latlng const& a, geo::latlng const& b, Fn&& fn) const {
    auto const min =
        std::array{std::min(a.lng_, b.lng_), std::min(a.lat_, b.lat_)};
    auto const max =
        std::array{std::max(a.lng_, b.lng_), std::max(a.lat_, b.lat_)};
    rtree_search(
        rtree_, min.data(), max.data(),
        [](double const* /* min */, double const* /* max */, void const* item,
           void* udata) {
          (*reinterpret_cast<Fn*>(udata))(
              way_idx_t{static_cast<way_idx_t::value_t>(
                  reinterpret_cast<std::size_t>(item))});
          return true;
        },
        &fn);
  }

  hash_set<node_idx_t> find_elevators(geo::latlng const& a,
                                      geo::latlng const& b) const {
    auto elevators = hash_set<node_idx_t>{};
    find(a, b, [&](way_idx_t const way) {
      for (auto const n : ways_.r_->way_nodes_[way]) {
        if (ways_.r_->node_properties_[n].is_elevator()) {
          elevators.emplace(n);
        }
      }
    });
    return elevators;
  }

  void insert(way_idx_t const way) {
    auto b = osmium::Box{};
    for (auto const& c : ways_.way_polylines_[way]) {
      b.extend(osmium::Location{c.lat_, c.lng_});
    }

    auto const min_corner =
        std::array{b.bottom_left().lon(), b.bottom_left().lat()};
    auto const max_corner =
        std::array{b.top_right().lon(), b.top_right().lat()};

    rtree_insert(
        rtree_, min_corner.data(), max_corner.data(),
        reinterpret_cast<void*>(static_cast<std::size_t>(to_idx(way))));
  }

  rtree* rtree_;
  ways const& ways_;
};

}  // namespace osr