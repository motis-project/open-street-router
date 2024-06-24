#pragma once

#include "osmium/osm/node.hpp"
#include "osmium/osm/relation.hpp"
#include "osmium/osm/way.hpp"

#include "rtree.h"

#include "utl/overloaded.h"
#include "utl/zip.h"

#include "osr/types.h"
#include "osr/ways.h"
#include "utl/parser/cstr.h"

namespace osr {

using platform_idx_t = cista::strong<std::uint32_t, struct platform_idx_>;

using ref_t = std::variant<way_idx_t, node_idx_t>;
using ref_value_t = std::uint32_t;

static_assert(sizeof(ref_value_t) >= sizeof(way_idx_t) &&
              sizeof(ref_value_t) >= sizeof(node_idx_t));

constexpr auto const kNodeMarker =
    ref_value_t{1U << (sizeof(ref_value_t) * 8U - 1U)};

constexpr bool is_way(ref_value_t const v) { return v < kNodeMarker; }

constexpr ref_t to_ref(ref_value_t const v) {
  if (is_way(v)) {
    return way_idx_t{v};
  } else {
    return node_idx_t{v - kNodeMarker};
  }
}

constexpr ref_value_t to_value(ref_t const r) {
  return std::visit(
      utl::overloaded{[](way_idx_t x) { return to_idx(x); },
                      [](node_idx_t x) { return to_idx(x) + kNodeMarker; }},
      r);
}

struct platforms {
  platforms(std::filesystem::path p, cista::mmap::protection const mode)
      : p_{std::move(p)},
        mode_{mode},
        node_pos_{mm("node_pos.bin")},
        node_is_platform_{mm_vec<std::uint64_t>{mm("node_is_platform.bin")}},
        way_is_platform_{mm_vec<std::uint64_t>{mm("way_is_platform.bin")}},
        platform_ref_{cista::paged<mm_vec32<ref_value_t>>{
                          mm_vec32<ref_value_t>{mm("platform_ref_data.bin")}},
                      mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
                          mm("platform_ref_index.bin")}},
        platform_names_{{mm_vec<std::uint64_t>{mm("platform_names_idx_0.bin")},
                         mm_vec<std::uint64_t>{mm("platform_names_idx_1.bin")}},
                        mm_vec<char>{mm("platform_names_data.bin")}} {}

  ~platforms() {
    if (rtree_ != nullptr) {
      rtree_free(rtree_);
    }
  }

  platform_idx_t way(way_idx_t const w, osmium::Way const& x) {
    assert(w != way_idx_t::invalid());
    assert(to_idx(w) < kNodeMarker);
    auto const p = add_names(x);
    if (p == platform_idx_t::invalid()) {
      return platform_idx_t::invalid();
    }
    way_is_platform_.resize(std::max(
        way_is_platform_.size(), static_cast<std::uint64_t>(to_idx(w) + 1U)));
    way_is_platform_.set(w, true);
    platform_ref_.emplace_back(std::initializer_list<ref_value_t>{to_value(w)});
    return p;
  }

  platform_idx_t node(node_idx_t const n, osmium::Node const& x) {
    assert(n != node_idx_t::invalid());
    assert(to_idx(n) < kNodeMarker);
    auto const p = add_names(x);

    if (p == platform_idx_t::invalid()) {
      return platform_idx_t::invalid();
    }

    node_pos_.emplace_back(n, point::from_location(x.location()));
    node_is_platform_.resize(std::max(
        node_is_platform_.size(), static_cast<std::uint64_t>(to_idx(n) + 1U)));
    node_is_platform_.set(n, true);
    platform_ref_.emplace_back(std::initializer_list<ref_value_t>{to_value(n)});
    return p;
  }

  platform_idx_t relation(osmium::Relation const& r) {
    auto const p = add_names(r);

    if (p == platform_idx_t::invalid()) {
      return platform_idx_t::invalid();
    }

    platform_ref_.emplace_back_empty();
    return p;
  }

  platform_idx_t add_names(osmium::OSMObject const& x) {
    strings_.clear();
    for (auto const& t : x.tags()) {
      switch (cista::hash(std::string_view{t.key()})) {
        case cista::hash("ref"):
        case cista::hash("ref:IFOPT"):
        case cista::hash("ref:operator"):
        case cista::hash("ref_name"):
        case cista::hash("local_ref"):
        case cista::hash("name"):
        case cista::hash("alt_name"): [[fallthrough]];
        case cista::hash("description"):
          utl::for_each_token(t.value(), ';', [&](auto const s) {
            strings_.emplace_back(s.view());
          });
      }
    }

    if (strings_.empty()) {
      return platform_idx_t::invalid();
    }

    auto const idx = platform_idx_t{platform_names_.size()};
    platform_names_.emplace_back(strings_);
    return idx;
  }

  level_t get_level(ways const& w, platform_idx_t const i) const {
    return std::visit(
        utl::overloaded{
            [&](way_idx_t x) { return w.r_->way_properties_[x].from_level(); },
            [&](node_idx_t x) {
              return w.r_->node_properties_[x].from_level();
            }},
        to_ref(platform_ref_[i][0]));
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
              platform_idx_t{static_cast<way_idx_t::value_t>(
                  reinterpret_cast<std::size_t>(item))});
          return true;
        },
        &fn);
  }

  point get_node_pos(node_idx_t const i) const {
    auto const it =
        std::lower_bound(begin(node_pos_), end(node_pos_), i,
                         [](auto&& a, auto&& b) { return a.first < b; });
    utl::verify(it != end(node_pos_) || it->first == i, "node pos not found");
    return it->second;
  }

  void build_rtree(ways const& w) {
    assert(rtree_ == nullptr);

    rtree_ = rtree_new();

    auto i = platform_idx_t{0U};
    for (auto const refs : platform_ref_) {
      for (auto const ref : refs) {
        assert(ref != std::numeric_limits<decltype(ref)>::max());
        std::visit(
            utl::overloaded{
                [&](node_idx_t const x) {
                  auto const pos = get_node_pos(x).as_latlng();
                  auto const min_corner = std::array{pos.lng(), pos.lat()};
                  rtree_insert(rtree_, min_corner.data(), nullptr,
                               reinterpret_cast<void*>(
                                   static_cast<std::size_t>(to_idx(i))));
                },

                [&](way_idx_t const x) {
                  auto b = osmium::Box{};
                  for (auto const& c : w.way_polylines_[x]) {
                    b.extend(osmium::Location{c.lat_, c.lng_});
                  }

                  auto const min_corner =
                      std::array{b.bottom_left().lon(), b.bottom_left().lat()};
                  auto const max_corner =
                      std::array{b.top_right().lon(), b.top_right().lat()};

                  rtree_insert(rtree_, min_corner.data(), max_corner.data(),
                               reinterpret_cast<void*>(
                                   static_cast<std::size_t>(to_idx(i))));
                }},
            to_ref(ref));
      }
      ++i;
    }
  }

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  mm_vec<pair<node_idx_t, point>> node_pos_;
  mm_bitvec<node_idx_t> node_is_platform_;
  mm_bitvec<way_idx_t> way_is_platform_;
  mm_paged_vecvec<platform_idx_t, ref_value_t> platform_ref_;
  mm_nvec<platform_idx_t, char, 2> platform_names_;

  vecvec<std::uint32_t, char> strings_;

  rtree* rtree_{nullptr};
};

}  // namespace osr