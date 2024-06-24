#pragma once

#include "cista/hash.h"

#include "osmium/osm/object.hpp"

#include "osr/types.h"

namespace osr {

enum class osm_obj_type : std::uint8_t { kWay, kNode };

enum class override : std::uint8_t { kNone, kWhitelist, kBlacklist };

struct tags {
  explicit tags(osmium::OSMObject const& o) {
    for (auto const& t : o.tags()) {
      switch (cista::hash(std::string_view{t.key()})) {
        using namespace std::string_view_literals;
        case cista::hash("building"): [[fallthrough]];
        case cista::hash("landuse"): landuse_ = true; break;
        case cista::hash("railway"):
          landuse_ |= t.value() == "station_area"sv;
          break;
        case cista::hash("oneway"): oneway_ |= t.value() == "yes"sv; break;
        case cista::hash("junction"):
          oneway_ |= t.value() == "roundabout"sv;
          break;
        case cista::hash("oneway:bicycle"):
          not_oneway_bike_ = t.value() == "no"sv;
          break;
        case cista::hash("motor_vehicle"):
          motor_vehicle_ = t.value();
          is_destination_ |= motor_vehicle_ == "destination";
          break;
        case cista::hash("foot"): foot_ = t.value(); break;
        case cista::hash("bicycle"): bicycle_ = t.value(); break;
        case cista::hash("highway"):
          highway_ = t.value();
          if (highway_ == "elevator") {
            is_elevator_ = true;
          }
          if (highway_ == "bus_stop") {
            is_platform_ = true;
          }
          break;
        case cista::hash("level"): {
          has_level_ = true;
          auto s = utl::cstr{t.value()};
          while (s) {
            auto l = 0.0F;
            utl::parse_arg(s, l);
            auto const lvl = to_level(std::clamp(l, kMinLevel, kMaxLevel));
            level_bits_ |= (1U << to_idx(lvl));
            if (s) {
              ++s;
            }
          }
          break;
        }
        case cista::hash("name"): name_ = t.value(); break;
        case cista::hash("entrance"): is_entrance_ = true; break;
        case cista::hash("sidewalk"): sidewalk_ = t.value(); break;
        case cista::hash("cycleway"): cycleway_ = t.value(); break;
        case cista::hash("motorcar"):
          motorcar_ = t.value();
          is_destination_ |= motorcar_ == "destination";
          break;
        case cista::hash("barrier"): barrier_ = t.value(); break;
        case cista::hash("platform_edge"): is_platform_ = true; break;
        case cista::hash("public_transport"):
          switch (cista::hash(std::string_view{t.value()})) {
            case cista::hash("platform"):
            case cista::hash("stop_position"):
            case cista::hash("stop_area"): is_platform_ = true;
          }
          break;
        case cista::hash("vehicle"):
          switch (cista::hash(std::string_view{t.value()})) {
            case cista::hash("private"):
            case cista::hash("delivery"):
            case cista::hash("no"): vehicle_ = override::kBlacklist; break;

            case cista::hash("destination"):
              is_destination_ = true;
              [[fallthrough]];
            case cista::hash("permissive"): [[fallthrough]];
            case cista::hash("yes"): vehicle_ = override::kWhitelist; break;
          }
          break;
        case cista::hash("access"):
          switch (cista::hash(std::string_view{t.value()})) {
            case cista::hash("no"):
            case cista::hash("agricultural"):
            case cista::hash("forestry"):
            case cista::hash("emergency"):
            case cista::hash("psv"):
            case cista::hash("private"): [[fallthrough]];
            case cista::hash("delivery"): access_ = override::kBlacklist; break;

            case cista::hash("designated"):
            case cista::hash("dismount"):
            case cista::hash("customers"):
            case cista::hash("permissive"): [[fallthrough]];
            case cista::hash("yes"): access_ = override::kWhitelist; break;
          }
          break;
        case cista::hash("maxspeed"): max_speed_ = t.value(); break;
      }
    }
  }

  // https://wiki.openstreetmap.org/wiki/Key:oneway
  // https://wiki.openstreetmap.org/wiki/Tag:junction=roundabout
  bool oneway_{false};

  // https://wiki.openstreetmap.org/wiki/Key:oneway:bicycle
  bool not_oneway_bike_{false};

  // https://wiki.openstreetmap.org/wiki/Key:barrier
  std::string_view barrier_;

  // https://wiki.openstreetmap.org/wiki/Key:motorcar
  std::string_view motorcar_;

  // https://wiki.openstreetmap.org/wiki/Key:motor_vehicle
  std::string_view motor_vehicle_;

  // https://wiki.openstreetmap.org/wiki/Key:foot
  std::string_view foot_;

  // https://wiki.openstreetmap.org/wiki/Key:bicycle
  std::string_view bicycle_;

  // https://wiki.openstreetmap.org/wiki/DE:Key:highway
  std::string_view highway_;

  // https://wiki.openstreetmap.org/wiki/Key:sidewalk
  std::string_view sidewalk_;

  // https://wiki.openstreetmap.org/wiki/Key:cycleway
  std::string_view cycleway_;

  // https://wiki.openstreetmap.org/wiki/Key:maxspeed
  std::string_view max_speed_;

  // https://wiki.openstreetmap.org/wiki/Key:name
  std::string_view name_;

  // https://wiki.openstreetmap.org/wiki/Key:vehicle
  bool is_destination_{false};
  override vehicle_{override::kNone};

  // https://wiki.openstreetmap.org/wiki/Key:access
  override access_{override::kNone};

  // https://wiki.openstreetmap.org/wiki/Key:landuse
  bool landuse_{false};

  // https://wiki.openstreetmap.org/wiki/Key:public%20transport
  bool is_platform_{false};

  // https://wiki.openstreetmap.org/wiki/Tag:highway=elevator
  bool is_elevator_{false};

  // https://wiki.openstreetmap.org/wiki/Key:entrance
  bool is_entrance_{false};

  // https://wiki.openstreetmap.org/wiki/Key:level
  bool has_level_{false};
  level_bits_t level_bits_{0U};
};

template <typename T>
bool is_accessible(tags const& o, osm_obj_type const type) {
  auto const override = T::access_override(o);
  return override == override::kWhitelist ||
         (T::default_access(o, type) && override != override::kBlacklist);
}

struct foot_profile {
  static override access_override(tags const& t) {
    switch (cista::hash(t.barrier_)) {
      case cista::hash("yes"):
      case cista::hash("wall"):
      case cista::hash("fence"): return override::kBlacklist;
    }

    switch (cista::hash(t.foot_)) {
      case cista::hash("no"):
      case cista::hash("private"): [[fallthrough]];
      case cista::hash("use_sidepath"): return override::kBlacklist;

      case cista::hash("yes"):
      case cista::hash("permissive"): [[fallthrough]];
      case cista::hash("designated"): return override::kWhitelist;
    }

    if (t.is_platform_) {
      return override::kWhitelist;
    }

    if (t.access_ == override::kBlacklist) {
      return override::kBlacklist;
    }

    return override::kNone;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      if (t.is_elevator_) {
        return true;
      }
      switch (cista::hash(t.highway_)) {
        case cista::hash("primary"):
        case cista::hash("primary_link"):
        case cista::hash("secondary"):
        case cista::hash("secondary_link"):
        case cista::hash("tertiary"):
        case cista::hash("tertiary_link"):
        case cista::hash("unclassified"):
        case cista::hash("residential"):
        case cista::hash("road"):
        case cista::hash("living_street"):
        case cista::hash("service"):
        case cista::hash("track"):
        case cista::hash("path"):
        case cista::hash("steps"):
        case cista::hash("pedestrian"):
        case cista::hash("platform"):
        case cista::hash("footway"): [[fallthrough]];
        case cista::hash("pier"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }
};

struct bike_profile {
  static override access_override(tags const& t) {

    switch (cista::hash(t.barrier_)) {
      case cista::hash("yes"):
      case cista::hash("wall"):
      case cista::hash("fence"): return override::kBlacklist;
    }

    switch (cista::hash(t.bicycle_)) {
      case cista::hash("no"):
      case cista::hash("private"):
      case cista::hash("optional_sidepath"): [[fallthrough]];
      case cista::hash("use_sidepath"): return override::kBlacklist;

      case cista::hash("yes"):
      case cista::hash("permissive"): [[fallthrough]];
      case cista::hash("designated"): return override::kWhitelist;
    }

    if (t.access_ == override::kBlacklist) {
      return override::kBlacklist;
    }

    return t.vehicle_;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      switch (cista::hash(t.highway_)) {
        case cista::hash("cycleway"):
        case cista::hash("primary"):
        case cista::hash("primary_link"):
        case cista::hash("secondary"):
        case cista::hash("secondary_link"):
        case cista::hash("tertiary"):
        case cista::hash("tertiary_link"):
        case cista::hash("residential"):
        case cista::hash("unclassified"):
        case cista::hash("living_street"):
        case cista::hash("road"):
        case cista::hash("service"):
        case cista::hash("track"):
        case cista::hash("path"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }
};

struct car_profile {
  static override access_override(tags const& t) {
    if (t.access_ == override::kBlacklist) {
      return override::kBlacklist;
    }

    if (!t.barrier_.empty()) {
      switch (cista::hash(t.barrier_)) {
        case cista::hash("cattle_grid"):
        case cista::hash("border_control"):
        case cista::hash("toll_booth"):
        case cista::hash("sally_port"):
        case cista::hash("gate"):
        case cista::hash("lift_gate"):
        case cista::hash("no"):
        case cista::hash("entrance"):
        case cista::hash("height_restrictor"): [[fallthrough]];
        case cista::hash("arch"): break;
        default: return override::kBlacklist;
      }
    }

    auto const get_override = [](std::string_view tag) {
      switch (cista::hash(tag)) {
        case cista::hash("private"):
        case cista::hash("optional_sidepath"):
        case cista::hash("agricultural"):
        case cista::hash("forestry"):
        case cista::hash("agricultural;forestry"):
        case cista::hash("permit"):
        case cista::hash("customers"):
        case cista::hash("delivery"): [[fallthrough]];
        case cista::hash("no"): return override::kBlacklist;

        case cista::hash("designated"):
        case cista::hash("permissive"): [[fallthrough]];
        case cista::hash("yes"): return override::kWhitelist;

        default: return override::kNone;
      }
    };

    if (auto mv = get_override(t.motor_vehicle_); mv != override::kNone) {
      return mv;
    } else if (auto mc = get_override(t.motorcar_); mv != override::kNone) {
      return mc;
    } else {
      return t.vehicle_;
    }
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      switch (cista::hash(t.highway_)) {
        case cista::hash("motorway"):
        case cista::hash("motorway_link"):
        case cista::hash("trunk"):
        case cista::hash("trunk_link"):
        case cista::hash("primary"):
        case cista::hash("primary_link"):
        case cista::hash("secondary"):
        case cista::hash("secondary_link"):
        case cista::hash("tertiary"):
        case cista::hash("tertiary_link"):
        case cista::hash("residential"):
        case cista::hash("living_street"):
        case cista::hash("unclassified"):
        case cista::hash("service"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }
};

}  // namespace osr