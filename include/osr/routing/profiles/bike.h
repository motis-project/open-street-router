#pragma once

#include "osr/ways.h"

namespace osr {

struct bike {
  static constexpr auto const kMaxMatchDistance = 100U;
  static constexpr auto const kOffroadPenalty = 1U;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid()};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node get_key() const noexcept { return *this; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << w.node_to_osm_[n_];
    }

    node_idx_t n_;
  };

  using key = node;

  struct entry {
    constexpr std::optional<node> pred(node) const noexcept {
      return pred_ == node_idx_t::invalid() ? std::nullopt
                                            : std::optional{node{pred_}};
    }
    constexpr cost_t cost(node) const noexcept { return cost_; }
    constexpr bool update(node, cost_t const c, node const pred) noexcept {
      if (c < cost_) {
        cost_ = c;
        pred_ = pred.n_;
        return true;
      }
      return false;
    }

    node_idx_t pred_{node_idx_t::invalid()};
    cost_t cost_{kInfeasible};
  };

  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    level_t lvl_;
    cost_t cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(node const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n.n_)));
    }
  };

  template <typename Fn>
  static void resolve(
      ways::routing const&, way_idx_t, node_idx_t const n, level_t, Fn&& f) {
    f(node{n});
  }

  template <typename Fn>
  static void resolve_all(ways::routing const&,
                          node_idx_t const n,
                          level_t,
                          Fn&& f) {
    f(node{n});
  }

  static bool is_reachable(
      ways::routing const&, node, way_idx_t, direction, direction) {
    return true;
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       Fn&& fn) {
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        auto const target_node = w.way_nodes_[way][to];
        if constexpr (WithBlocked) {
          if (blocked->test(target_node)) {
            return;
          }
        }
        auto const target_node_prop = w.node_properties_[target_node];
        if (node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const cost = way_cost(target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop);
        fn(node{target_node}, static_cast<std::uint32_t>(cost), dist, way, from,
           to);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }
    }
  }

  static constexpr cost_t way_cost(way_properties const e,
                                   direction,
                                   std::uint16_t const dist) {
    if (e.is_bike_accessible()) {
      return static_cast<cost_t>(std::round(dist / 2.8F));
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const n) {
    return n.is_bike_accessible() ? 0U : kInfeasible;
  }
};

}  // namespace osr