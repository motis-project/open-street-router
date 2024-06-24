#pragma once

#include <cinttypes>
#include <filesystem>

#include "ankerl/cista_adapter.h"

#include "cista/containers/bitvec.h"
#include "cista/containers/mmap_vec.h"
#include "cista/containers/nvec.h"
#include "cista/containers/paged.h"
#include "cista/containers/paged_vecvec.h"
#include "cista/containers/vector.h"
#include "cista/containers/vecvec.h"
#include "cista/strong.h"

namespace osr {

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using vecvec = cista::raw::vecvec<K, V, SizeType>;

template <typename T>
using vec = cista::raw::vector<T>;

template <typename K, typename V>
using vec_map = cista::raw::vector_map<K, V>;

template <typename K>
using bitvec = cista::basic_bitvec<vec<std::uint64_t>, K>;

template <typename K, typename V>
using mm_vec_map = cista::basic_mmap_vec<V, K>;

template <typename T>
using mm_vec = cista::basic_mmap_vec<T, std::uint64_t>;

template <typename T>
using mm_vec32 = cista::basic_mmap_vec<T, std::uint32_t>;

template <typename K>
using mm_bitvec = cista::basic_bitvec<mm_vec<std::uint64_t>, K>;

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using mm_vecvec = cista::basic_vecvec<K, mm_vec<V>, mm_vec<SizeType>>;

template <typename K, typename V, std::size_t N>
using mm_nvec =
    cista::basic_nvec<K, mm_vec<V>, mm_vec<std::uint64_t>, N, std::uint64_t>;

template <typename Key, typename T>
struct mmap_paged_vecvec_helper {
  using data_t = cista::paged<mm_vec32<T>>;
  using idx_t = mm_vec<typename data_t::page_t>;
  using type = cista::paged_vecvec<idx_t, data_t, Key>;
};

template <typename Key, typename T>
using mm_paged_vecvec = mmap_paged_vecvec_helper<Key, T>::type;

using bitvec64 = cista::basic_bitvec<
    cista::basic_vector<std::uint64_t, cista::raw::ptr, false, std::uint64_t>>;

template <typename First, typename Second>
using pair = cista::pair<First, Second>;

template <typename K,
          typename V,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_map = cista::raw::ankerl_map<K, V, Hash, Equality>;

template <typename K,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_set = cista::raw::ankerl_set<K, Hash, Equality>;

using osm_node_idx_t = cista::strong<std::uint64_t, struct osm_node_idx_>;
using osm_way_idx_t = cista::strong<std::uint64_t, struct osm_way_idx_>;

using way_idx_t = cista::strong<std::uint32_t, struct way_idx_>;
using node_idx_t = cista::strong<std::uint32_t, struct node_idx_>;

using multi_level_elevator_idx_t =
    cista::strong<std::uint32_t, struct multi_level_elevator_idx_>;

using distance_t = std::uint16_t;

using way_pos_t = std::uint8_t;

using cost_t = std::uint16_t;

constexpr auto const kInfeasible = std::numeric_limits<cost_t>::max();

// direction
enum class direction : std::uint8_t {
  kForward,
  kBackward,
};

constexpr direction opposite(direction const dir) {
  return dir == direction::kForward ? direction::kBackward
                                    : direction::kForward;
}

template <direction Dir>
constexpr direction flip(direction const dir) {
  return Dir == direction::kForward ? dir : opposite(dir);
}

constexpr direction flip(direction const search_dir, direction const dir) {
  return search_dir == direction::kForward ? dir : opposite(dir);
}

constexpr std::string_view to_str(direction const d) {
  switch (d) {
    case direction::kForward: return "forward";
    case direction::kBackward: return "backward";
  }
  std::unreachable();
}

constexpr direction to_direction(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("forward"): return direction::kForward;
    case cista::hash("backward"): return direction::kBackward;
  }
  std::unreachable();
}

// level
using level_t = cista::strong<std::uint8_t, struct level_>;

constexpr auto const kMinLevel = -4.0F;
constexpr auto const kMaxLevel = 3.75F;

constexpr level_t to_level(float const f) {
  return level_t{static_cast<std::uint8_t>((f - kMinLevel) / 0.25F)};
}

constexpr float to_float(level_t const l) {
  return kMinLevel + (to_idx(l) / 4.0F);
}

constexpr auto const kLevelBits = cista::constexpr_trailing_zeros(
    cista::next_power_of_two(to_idx(to_level(kMaxLevel))));

using level_bits_t = std::uint32_t;

constexpr bool has_bit_set(level_bits_t const levels, unsigned const bit) {
  return (levels & (level_bits_t{1U} << bit)) != 0U;
}

template <typename Fn>
constexpr void for_each_set_bit(level_bits_t const levels, Fn&& f) {
  for (auto bit = 0U; bit != sizeof(level_bits_t) * 8U; ++bit) {
    if (has_bit_set(levels, bit)) {
      f(bit);
    }
  }
}

constexpr std::tuple<level_t, level_t, bool> get_levels(
    bool const has_level, level_bits_t const levels) noexcept {
  if (!has_level) {
    return {level_t{to_level(0.F)}, level_t{to_level(0.F)}, false};
  }
  auto from = level_t::invalid(), to = level_t::invalid();
  for_each_set_bit(levels, [&](auto&& bit) {
    from == level_t::invalid()  //
        ? from = level_t{bit}
        : to = level_t{bit};
  });
  return {from, to == level_t::invalid() ? from : to,
          std::popcount(levels) > 2};
}

static_assert(kLevelBits == 5U);

// speed
enum speed_limit : std::uint8_t {
  kmh_10,
  kmh_30,
  kmh_50,
  kmh_70,
  kmh_100,
  kmh_120,
};

constexpr speed_limit get_speed_limit(unsigned const x) {
  if (x >= 120U) {
    return speed_limit::kmh_120;
  } else if (x >= 100) {
    return speed_limit::kmh_100;
  } else if (x >= 70) {
    return speed_limit::kmh_70;
  } else if (x >= 50) {
    return speed_limit::kmh_50;
  } else if (x >= 30) {
    return speed_limit::kmh_30;
  } else {
    return speed_limit::kmh_10;
  }
}

constexpr std::uint16_t to_kmh(speed_limit const l) {
  switch (l) {
    case speed_limit::kmh_10: return 10U;
    case speed_limit::kmh_30: return 30U;
    case speed_limit::kmh_50: return 50U;
    case speed_limit::kmh_70: return 70U;
    case speed_limit::kmh_100: return 100U;
    case speed_limit::kmh_120: return 120U;
  }
  std::unreachable();
}

constexpr std::uint16_t to_meters_per_second(speed_limit const l) {
  return static_cast<std::uint16_t>(to_kmh(l) / 3.6);
}

}  // namespace osr