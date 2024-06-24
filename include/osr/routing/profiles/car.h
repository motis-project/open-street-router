#pragma once

#include <bitset>

#include "utl/helpers/algorithm.h"

#include "osr/ways.h"

namespace osr {

struct car {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{120U};

  using key = node_idx_t;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return node{
          .n_ = node_idx_t::invalid(), .way_ = 0U, .dir_ = direction::kForward};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ", dir=" << to_str(dir_)
                 << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                 << ")";
    }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
  };

  struct entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U /* FWD+BWD */;

    entry() { utl::fill(cost_, kInfeasible); }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_way_[idx],
                                      to_dir(pred_dir_[idx])}};
    }

    constexpr cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    constexpr bool update(node const n,
                          cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        return true;
      }
      return false;
    }

    static constexpr node get_node(node_idx_t const n,
                                   std::size_t const index) {
      return node{n, static_cast<way_pos_t>(index % kMaxWays),
                  to_dir((index / kMaxWays) == 0U)};
    }

    static constexpr std::size_t get_index(node const n) {
      return (n.dir_ == direction::kForward ? 0 : 1) * kMaxWays + n.way_;
    }

    static constexpr direction to_dir(bool const b) {
      return b == false ? direction::kForward : direction::kBackward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kForward ? false : true;
    }

    std::array<node_idx_t, kN> pred_;
    std::array<way_pos_t, kN> pred_way_;
    std::bitset<kN> pred_dir_;
    std::array<cost_t, kN> cost_;
  };

  struct label {
    label(node const n, cost_t const c)
        : n_{n.n_}, way_{n.way_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, way_, dir_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
    cost_t cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  template <typename Fn>
  static void resolve(ways::routing const& w,
                      way_idx_t const way,
                      node_idx_t const n,
                      level_t,
                      Fn&& f) {
    auto const ways = w.node_ways_[n];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      if (ways[i] == way) {
        f(node{n, i, direction::kForward});
        f(node{n, i, direction::kBackward});
      }
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      f(node{n, i, direction::kForward});
      f(node{n, i, direction::kBackward});
    }
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       Fn&& fn) {
    auto way_pos = way_pos_t{0U};
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
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

        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }

        auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const target =
            node{target_node, w.get_way_pos(target_node, way), way_dir};
        auto const cost = way_cost(target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop) +
                          (is_u_turn ? kUturnPenalty : 0U);
        fn(target, cost, dist, way, from, to);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }

      ++way_pos;
    }
  }

  static bool is_reachable(ways::routing const& w,
                           node const n,
                           way_idx_t const way,
                           direction const way_dir,
                           direction const search_dir) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    if (w.is_restricted(n.n_, n.way_, w.get_way_pos(n.n_, way), search_dir)) {
      return false;
    }

    return true;
  }

  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s()) * (e.is_destination() ? 5U : 1U) +
             (e.is_destination() ? 120U : 0U);
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }
};

}  // namespace osr