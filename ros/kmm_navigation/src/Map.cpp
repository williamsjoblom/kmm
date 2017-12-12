#include "kmm_navigation/Map.hpp"

namespace kmm_navigation {

  Map::Map(int map_rows, int map_cols, float cell_size)
  : h_(map_rows),
    w_(map_cols),
    cs_(cell_size) {
    offset_ = (w_ - 1) / 2;
    walls_size_ = (w_ + (w_ + 1)) * h_ + w_;

    // Initialize wall vector with 0's.
		for (int i = 0; i < walls_size_; i++) {
			walls_.push_back(0);
    };
  }

  Map::~Map() {
  }

  int Map::get_rows() {
    return h_;
  }

  int Map::get_cols() {
    return w_;
  }

  int Map::get_offset() {
    return offset_;
  }

  float Map::get_cell_size() {
    return cs_;
  }

  void Map::set_walls(std::vector<int> walls) {
    walls_ = walls;
  }

  Eigen::Vector2f Map::get_cell(Eigen::Vector2f grid_pos) {
    float x = grid_pos.x();
    float y = grid_pos.y();
    float cell_x = ((x - (cs_/2)) - remainder(x - (cs_/2), cs_)) / cs_;
    float cell_y = ((y - (cs_/2)) - remainder(y - (cs_/2), cs_)) / cs_;
    Eigen::Vector2f cell_pos;
    cell_pos[0] = cell_x;
    cell_pos[1] = cell_y;
    return cell_pos;
  }

  bool Map::is_cell_within_bounds(Eigen::Vector2f cell) {
    int index_row = (int)cell.x();
    int index_col = (int)cell.y() + get_offset();

    bool row_ok = index_row >= 0 && index_row < get_rows();
    bool col_ok = index_col >= 0 && index_col <= get_offset(); // (w_ - 1) / 2

    bool is_cell_within_bounds = row_ok && col_ok;

    return is_cell_within_bounds;
  }

  bool Map::is_wall_north_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = (x + 1)*(w_ + 1) + (x + 1)*w_ + offset_ + y;
    if (index >= walls_size_) {
      return true;
    }
    return Map::walls_[index];
  }

  bool Map::is_wall_south_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + x*w_ + offset_ + y;
    if (index >= walls_size_) {
      return true;
    }
    return walls_[index];
  }

  bool Map::is_wall_east_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + (x + 1)*w_ + offset_ + y;
    if (index >= walls_size_) {
      return true;
    }
    return walls_[index];
  }

  bool Map::is_wall_west_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + (x + 1)*w_ + offset_ + y + 1;
    if (index >= walls_size_) {
      return true;
    }
    return walls_[index];
  }

  bool Map::is_north_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_north_of_cell(cell) && cell.x() < (h_ - 1);
  }

  bool Map::is_north_east_reachable_from_cell(Eigen::Vector2f cell) {
    Eigen::Vector2f north_cell(cell.x() + 1, cell.y());
    bool is_path_north = is_north_reachable_from_cell(cell) && is_east_reachable_from_cell(north_cell);
    Eigen::Vector2f east_cell(cell.x(), cell.y() - 1);
    bool is_path_east = is_east_reachable_from_cell(cell) && is_north_reachable_from_cell(east_cell);
    return is_path_north && is_path_east;
  }

  bool Map::is_east_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_east_of_cell(cell) && cell.y() > -offset_;
  }

  bool Map::is_south_east_reachable_from_cell(Eigen::Vector2f cell) {
    Eigen::Vector2f south_cell(cell.x() - 1, cell.y());
    bool is_path_south = is_south_reachable_from_cell(cell) && is_east_reachable_from_cell(south_cell);
    Eigen::Vector2f east_cell(cell.x(), cell.y() - 1);
    bool is_path_east = is_east_reachable_from_cell(cell) && is_south_reachable_from_cell(east_cell);
    return is_path_south && is_path_east;
  }

  bool Map::is_south_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_south_of_cell(cell) && cell.x() > 0;
  }

  bool Map::is_south_west_reachable_from_cell(Eigen::Vector2f cell) {
    Eigen::Vector2f south_cell(cell.x() - 1, cell.y());
    bool is_path_south = is_south_reachable_from_cell(cell) && is_west_reachable_from_cell(south_cell);
    Eigen::Vector2f west_cell(cell.x(), cell.y() + 1);
    bool is_path_west = is_west_reachable_from_cell(cell) && is_south_reachable_from_cell(west_cell);
    return is_path_south && is_path_west;
  }

  bool Map::is_west_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_west_of_cell(cell) && cell.y() < offset_;
  }

  bool Map::is_north_west_reachable_from_cell(Eigen::Vector2f cell) {
    Eigen::Vector2f north_cell(cell.x() + 1, cell.y());
    bool is_path_north = is_north_reachable_from_cell(cell) && is_west_reachable_from_cell(north_cell);
    Eigen::Vector2f west_cell(cell.x(), cell.y() + 1);
    bool is_path_west = is_west_reachable_from_cell(cell) && is_north_reachable_from_cell(west_cell);
    return is_path_north && is_path_west;
  }

  bool Map::is_north_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) + 1 == round(cell_2.x())
      && round(cell_1.y()) == round(cell_2.y());
  }

  bool Map::is_north_east_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) + 1 == round(cell_2.x())
      && round(cell_1.y()) - 1 == round(cell_2.y());
  }

  bool Map::is_east_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) == round(cell_2.x())
      && round(cell_1.y()) - 1 == round(cell_2.y());
  }

  bool Map::is_south_east_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) - 1 == round(cell_2.x())
      && round(cell_1.y()) - 1 == round(cell_2.y());
  }

  bool Map::is_south_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) - 1 == round(cell_2.x())
      && round(cell_1.y()) == round(cell_2.y());
  }

  bool Map::is_south_west_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) - 1 == round(cell_2.x())
      && round(cell_1.y()) + 1 == round(cell_2.y());
  }

  bool Map::is_west_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) == round(cell_2.x())
      && round(cell_1.y()) + 1 == round(cell_2.y());
  }

  bool Map::is_north_west_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2) {
    return round(cell_1.x()) + 1 == round(cell_2.x())
      && round(cell_1.y()) + 1 == round(cell_2.y());
  }

  bool Map::is_wall_in_path(const std::vector<Eigen::Vector2f>& path) {

    if (path.size() < 2) {
      return false;
    }

    Eigen::Vector2f curr_pos;
    Eigen::Vector2f next_pos;

    Eigen::Vector2f curr_cell;
    Eigen::Vector2f next_cell;

    bool is_next_reachable;

    for (int i = 0; i < path.size() - 1; i ++) {
      curr_pos = path[i];
      next_pos = path[i + 1];

      curr_cell = get_cell(curr_pos);
      next_cell = get_cell(next_pos);

      if (is_north_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_north_reachable_from_cell(curr_cell);

      } else if (is_north_east_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_north_east_reachable_from_cell(curr_cell);

      } else if (is_east_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_east_reachable_from_cell(curr_cell);

      } else if (is_south_east_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_south_east_reachable_from_cell(curr_cell);

      } else if (is_south_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_south_reachable_from_cell(curr_cell);

      } else if (is_south_west_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_south_west_reachable_from_cell(curr_cell);

      } else if (is_west_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_west_reachable_from_cell(curr_cell);

      } else if (is_north_west_of_cell(curr_cell, next_cell)) {
        is_next_reachable = is_north_west_reachable_from_cell(curr_cell);
      }

      if (!is_next_reachable) {
        return true;
      }

    }
    return false;
  }
}
