#pragma once

#include <filesystem>

#include "osmium/osm/way.hpp"

#include "geo/latlng.h"

#include "osr/node_info.h"
#include "osr/point.h"
#include "osr/types.h"
#include "osr/way_info.h"

namespace osr {

struct db {
  db(std::filesystem::path const&);
  ~db();

  point get_node_pos(osm_node_idx_t);
  void get_node(osm_node_idx_t, node_info&);
  void get_way(osm_way_idx_t, way_info&);

  void write(hash_map<osm_way_idx_t, way_info>&);
  void write(hash_map<osm_node_idx_t, node_info>&, bool create);

  void write_graph();

  void write_debug(std::ostream&);

  void flush();

  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr