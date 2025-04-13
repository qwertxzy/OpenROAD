// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2018-2025, The OpenROAD Authors

#include "dpl/Opendp.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "PlacementDRC.h"
#include "boost/geometry/geometry.hpp"
#include "dpl/OptMirror.h"
#include "graphics/DplObserver.h"
#include "infrastructure/Coordinates.h"
#include "infrastructure/DecapObjects.h"
#include "infrastructure/Grid.h"
#include "infrastructure/Objects.h"
#include "infrastructure/Padding.h"
#include "infrastructure/network.h"
#include "odb/db.h"
#include "odb/util.h"
#include "util/journal.h"
#include "utl/Logger.h"

namespace dpl {

using std::round;
using std::string;

using utl::DPL;

using odb::dbInst;
using odb::Rect;
using odb::Vector2D;

using utl::format_as;

////////////////////////////////////////////////////////////////

bool Opendp::isMultiRow(const Node* cell) const
{
  return network_->getMaster(cell->getDbInst()->getMaster())->isMultiRow();
}

////////////////////////////////////////////////////////////////

Opendp::Opendp(odb::dbDatabase* db, Logger* logger) : logger_(logger), db_(db)
{
  dummy_cell_ = std::make_unique<Node>();
  dummy_cell_->setPlaced(true);
  padding_ = std::make_shared<Padding>();
  grid_ = std::make_unique<Grid>();
  grid_->init(logger);
  network_ = std::make_unique<Network>();
  arch_ = std::make_unique<Architecture>();
}

// LEO: Method to unplace std cells
void Opendp::unplaceStdCells() {
  importDb();
  for (Cell& cell : cells_) {
    // Unplace all std cells
    if (cell.isStdCell()) {
      cell.db_inst_->setPlacementStatus(odb::dbPlacementStatus::UNPLACED);
    }
  }
}

// LEO: Method to fix slight overlaps between macros after GPL
// Plan:
//  - Every macro has a spring force to its original position
//  - Every macro has a repulsive force for each overlap
//  - Iterate until overlaps are resolved or max_iter is reached
// TODO: big halos will solve overlaps but overlap area won't go to 0?
void Opendp::fixMacroPlacement(float overlap_multiplier, float origin_multiplier, float damping_factor, int halo_width, int max_iter) {
  importDb();

  // Get a list of all macros
  std::vector<dbInst*> macros;
  std::map<dbInst*, Point> original_coordinates;
  for (Cell& cell : cells_) {
    if (!cell.isStdCell()) {
      macros.push_back(cell.db_inst_);
      original_coordinates[cell.db_inst_] = cell.db_inst_->getOrigin();

      // While we're at it, set all locked macros to just 'placed'
      if (cell.db_inst_->getPlacementStatus() == odb::dbPlacementStatus::LOCKED) {
        cell.db_inst_->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      }
    }
  }
  
  // Save force vectors for every macro
  std::map<dbInst*, Vector2D> forces = std::map<dbInst*, Vector2D>();

  // Initialize forces with a spring to each macro's original position
  for (auto& macro : macros) {
    Point original_position = original_coordinates[macro];
    Point macro_pos = macro->getOrigin();
    Vector2D spring_vector = Vector2D(macro_pos, original_position);

    // Add multiplied spring force to forces map
    forces[macro] = spring_vector * origin_multiplier;
  }

  // Save total overlap for this iteration
  int64_t total_overlap;

  for (int iteration = 0; iteration < max_iter; iteration++) {
    total_overlap = 0;
    // Loop over all macros i
    for (int i = 0; i < macros.size(); i++) {
      dbInst* macro_i = macros[i];
      Rect rect_i;
      if (halo_width > 0) {
        macro_i->getBBox()->getBox().bloat(halo_width, rect_i);
      } else {
        rect_i = macro_i->getBBox()->getBox();
      }     

      // Loop over all other macros j
      for (int j = i + 1; j < macros.size(); j++) {
        dbInst* macro_j = macros[j];
        Rect rect_j;
        if (halo_width > 0) {
          macro_j->getBBox()->getBox().bloat(halo_width, rect_j);
        } else {
          rect_j = macro_j->getBBox()->getBox();
        }

        // Check if overlap is nonzero
        if (rect_i.intersects(rect_j)) {
          // Direction is vector from center i to center j
          Vector2D direction = Vector2D(rect_i.center(), rect_j.center());
          direction.normalize();

          // Calculate overlap distances
          Rect overlap_rect = rect_i.intersect(rect_j);
          int overlap_dist = std::min(overlap_rect.dx(), overlap_rect.dy());
        
          debugPrint(logger_,
            DPL,
            "macro",
            2,
            "Overlap between {} and {}: overlap distance = {}",
            macro_i->getName(),
            macro_j->getName(),
            overlap_dist
          );

          // Calculate force magnitude
          int force_magnitude = overlap_multiplier * overlap_dist;

          // Calculate additional damping term
          float force_damping = -damping_factor * sqrt(force_magnitude);

          // Create a vector from these
          Vector2D force_vector = direction * force_magnitude + Vector2D(force_damping, force_damping);

          debugPrint(logger_,
            DPL,
            "macro",
            2,
            "Calculated force vector between {} and {}: ({}, {})",
            macro_i->getName(),
            macro_j->getName(),
            force_vector.getX(),
            force_vector.getY()
          );

          // Add vector to macro_j and subtract from macro_i
          forces[macro_j] = forces[macro_j] + force_vector;
          forces[macro_i] = forces[macro_i] - force_vector;

          // Update total overlap
          total_overlap += overlap_rect.area();
        }
      }
    }

    logger_->info(DPL, 3, "Iteration {}: total overlap = {}", iteration, total_overlap);

    // Apply forces to all macros
    Rect core = grid_->getCore();
    for (auto& force : forces) {
      dbInst* macro = force.first;
      Vector2D force_vector = force.second;
      if (force_vector.getMagnitude() == 0) {
        continue;
      }
      
      debugPrint(logger_,
        DPL,
        "macro",
        1,
        "Macro {}: force vector = ({}, {})",
        macro->getName(),
        force_vector.getX(),
        force_vector.getY()
      );

      // Move macro by force vector
      Point current_pos = macro->getOrigin();
      current_pos.addX(force_vector.getX());
      current_pos.addY(force_vector.getY());
      
      // Clip out of bounds coordinates
      if (current_pos.getX() < core.xMin()) {
        current_pos.setX(core.xMin());
      } else if (current_pos.getX() > core.xMax() - macro->getBBox()->getDX()) {
        current_pos.setX(core.xMax() - macro->getBBox()->getDX());
      }

      if (current_pos.getY() < core.yMin()) {
        current_pos.setY(core.yMin());
      } else if (current_pos.getY() > core.yMax() - macro->getBBox()->getDY()) {
        current_pos.setY(core.yMax() - macro->getBBox()->getDY());
      }

      // Set new position
      macro->setOrigin(current_pos.getX(), current_pos.getY());
    }

    // If total overlap area was 0, break out of the loop
    if (total_overlap == 0) {
      logger_->info(DPL, 4, "Resolved all overlaps in {} iterations", iteration);

      // TODO: Snap all macros to manufacturing grid
      // for (auto& macro : macros) {
      //   Snapper snapper(logger_, macro->db_inst_);
      //   snapper.snapMacro();
      //   macro->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
      // }

      break;
    }

    // Clear forces for next iteration
    forces.clear();
  }
}

Opendp::~Opendp() = default;

void Opendp::setPaddingGlobal(const int left, const int right)
{
  padding_->setPaddingGlobal(GridX{left}, GridX{right});
}

void Opendp::setPadding(odb::dbInst* inst, const int left, const int right)
{
  padding_->setPadding(inst, GridX{left}, GridX{right});
}

void Opendp::setPadding(odb::dbMaster* master, const int left, const int right)
{
  padding_->setPadding(master, GridX{left}, GridX{right});
}

void Opendp::setDebug(std::unique_ptr<DplObserver>& observer)
{
  debug_observer_ = std::move(observer);
}

void Opendp::setJournal(Journal* journal)
{
  journal_ = journal;
}

Journal* Opendp::getJournal() const
{
  return journal_;
}

void Opendp::detailedPlacement(const int max_displacement_x,
                               const int max_displacement_y,
                               const std::string& report_file_name)
{
  importDb();
  adjustNodesOrient();
  for (const auto& node : network_->getNodes()) {
    if (node->getType() == Node::CELL && !node->isFixed()) {
      node->setPlaced(false);
    }
  }

  if (have_fillers_) {
    logger_->warn(DPL, 37, "Use remove_fillers before detailed placement.");
  }

  if (max_displacement_x == 0 || max_displacement_y == 0) {
    // defaults
    max_displacement_x_ = 500;
    max_displacement_y_ = 100;
  } else {
    max_displacement_x_ = max_displacement_x;
    max_displacement_y_ = max_displacement_y;
  }

  odb::WireLengthEvaluator eval(block_);
  hpwl_before_ = eval.hpwl();
  detailedPlacement();
  // Save displacement stats before updating instance DB locations.
  findDisplacementStats();
  updateDbInstLocations();
  if (!placement_failures_.empty()) {
    logger_->info(DPL,
                  34,
                  "Detailed placement failed on the following {} instances:",
                  placement_failures_.size());
    for (auto cell : placement_failures_) {
      logger_->info(DPL, 35, " {}", cell->name());
    }

    saveFailures({}, {}, {}, {}, {}, {}, {}, placement_failures_, {}, {});
    if (!report_file_name.empty()) {
      writeJsonReport(report_file_name);
    }
    logger_->error(DPL, 36, "Detailed placement failed.");
  }
}

void Opendp::updateDbInstLocations()
{
  for (auto& cell : network_->getNodes()) {
    if (!cell->isFixed() && cell->isStdCell()) {
      odb::dbInst* db_inst_ = cell->getDbInst();
      // Only move the instance if necessary to avoid triggering callbacks.
      if (db_inst_->getOrient() != cell->getOrient()) {
        db_inst_->setOrient(cell->getOrient());
      }
      const DbuX x = core_.xMin() + cell->getLeft();
      const DbuY y = core_.yMin() + cell->getBottom();
      int inst_x, inst_y;
      db_inst_->getLocation(inst_x, inst_y);
      if (x != inst_x || y != inst_y) {
        db_inst_->setLocation(x.v, y.v);
      }
    }
  }
}

void Opendp::reportLegalizationStats() const
{
  logger_->report("Placement Analysis");
  logger_->report("---------------------------------");
  logger_->report("total displacement   {:10.1f} u",
                  block_->dbuToMicrons(displacement_sum_));
  logger_->metric("design__instance__displacement__total",
                  block_->dbuToMicrons(displacement_sum_));
  logger_->report("average displacement {:10.1f} u",
                  block_->dbuToMicrons(displacement_avg_));
  logger_->metric("design__instance__displacement__mean",
                  block_->dbuToMicrons(displacement_avg_));
  logger_->report("max displacement     {:10.1f} u",
                  block_->dbuToMicrons(displacement_max_));
  logger_->metric("design__instance__displacement__max",
                  block_->dbuToMicrons(displacement_max_));
  logger_->report("original HPWL        {:10.1f} u",
                  block_->dbuToMicrons(hpwl_before_));
  odb::WireLengthEvaluator eval(block_);
  const double hpwl_legal = eval.hpwl();
  logger_->report("legalized HPWL       {:10.1f} u",
                  block_->dbuToMicrons(hpwl_legal));
  logger_->metric("route__wirelength__estimated",
                  block_->dbuToMicrons(hpwl_legal));
  const int hpwl_delta
      = (hpwl_before_ == 0.0)
            ? 0.0
            : round((hpwl_legal - hpwl_before_) / hpwl_before_ * 100);
  logger_->report("delta HPWL           {:10} %", hpwl_delta);
  logger_->report("");
}

////////////////////////////////////////////////////////////////

void Opendp::findDisplacementStats()
{
  displacement_avg_ = 0;
  displacement_sum_ = 0;
  displacement_max_ = 0;
  for (auto& cell : network_->getNodes()) {
    if (cell->getType() != Node::CELL) {
      continue;
    }
    const int displacement = disp(cell.get());
    displacement_sum_ += displacement;
    if (displacement > displacement_max_) {
      displacement_max_ = displacement;
    }
  }
  if (network_->getNumCells() != 0) {
    displacement_avg_ = displacement_sum_ / network_->getNumCells();
  } else {
    displacement_avg_ = 0.0;
  }
}

////////////////////////////////////////////////////////////////

void Opendp::optimizeMirroring()
{
  OptimizeMirroring opt(logger_, db_);
  opt.run();
}

int Opendp::disp(const Node* cell) const
{
  const DbuPt init = initialLocation(cell, false);
  return sumXY(abs(init.x - cell->getLeft()), abs(init.y - cell->getBottom()));
}

int Opendp::padGlobalLeft() const
{
  return padding_->padGlobalLeft().v;
}

int Opendp::padGlobalRight() const
{
  return padding_->padGlobalRight().v;
}

int Opendp::padLeft(odb::dbInst* inst) const
{
  return padding_->padLeft(inst).v;
}

int Opendp::padRight(odb::dbInst* inst) const
{
  return padding_->padRight(inst).v;
}

void Opendp::initGrid()
{
  grid_->initGrid(
      db_, block_, padding_, max_displacement_x_, max_displacement_y_);
}

void Opendp::deleteGrid()
{
  grid_->clear();
}

void Opendp::findOverlapInRtree(const bgBox& queryBox,
                                std::vector<bgBox>& overlaps) const
{
  overlaps.clear();
  regions_rtree_.query(boost::geometry::index::intersects(queryBox),
                       std::back_inserter(overlaps));
}

void Opendp::setFixedGridCells()
{
  for (auto& cell : network_->getNodes()) {
    if (cell->getType() == Node::CELL && cell->isFixed()) {
      grid_->visitCellPixels(*cell, true, [&](Pixel* pixel, bool padded) {
        if (padded) {
          pixel->padding_reserved_by = cell.get();
        } else {
          setGridCell(*cell, pixel);
        }
      });
    }
  }
}

void Opendp::setGridCell(Node& cell, Pixel* pixel)
{
  pixel->cell = &cell;
  pixel->util = 1.0;
  if (cell.isBlock()) {
    // Try the is_hopeless strategy to get off of a block
    pixel->is_hopeless = true;
  }
}

void Opendp::groupAssignCellRegions()
{
  const int64_t site_width = grid_->getSiteWidth().v;
  const GridX row_site_count = grid_->getRowSiteCount();
  const GridY row_count = grid_->getRowCount();

  for (auto& group : arch_->getRegions()) {
    int64_t total_site_area = 0;
    if (!group->getCells().empty()) {
      for (GridX x{0}; x < row_site_count; x++) {
        for (GridY y{0}; y < row_count; y++) {
          const Pixel* pixel = grid_->gridPixel(x, y);
          if (pixel->is_valid && pixel->group == group) {
            total_site_area += grid_->rowHeight(y).v * site_width;
          }
        }
      }
    }

    double cell_area = 0;
    for (Node* cell : group->getCells()) {
      cell_area += cell->area();

      for (const auto& rect : group->getRects()) {
        if (isInside(cell, rect)) {
          cell->setRegion(&rect);
        }
      }
      if (cell->getRegion() == nullptr) {
        cell->setRegion(group->getRects().data());
      }
    }
    group->setUtil(total_site_area ? cell_area / total_site_area : 0.0);
  }
}

void Opendp::groupInitPixels2()
{
  for (GridX x{0}; x < grid_->getRowSiteCount(); x++) {
    for (GridY y{0}; y < grid_->getRowCount(); y++) {
      const Rect sub(x.v * grid_->getSiteWidth().v,
                     grid_->gridYToDbu(y).v,
                     (x + 1).v * grid_->getSiteWidth().v,
                     grid_->gridYToDbu(y + 1).v);
      Pixel* pixel = grid_->gridPixel(x, y);
      for (auto& group : arch_->getRegions()) {
        for (const Rect& rect : group->getRects()) {
          if (!isInside(sub, rect) && checkOverlap(sub, rect)) {
            pixel->util = 0.0;
            pixel->cell = dummy_cell_.get();
            pixel->is_valid = false;
            debugPrint(logger_,
                       DPL,
                       "group",
                       1,
                       "Block pixel [({}, {}) on region boundary",
                       x.v,
                       y.v);
          }
        }
      }
    }
  }
}

odb::dbInst* Opendp::getAdjacentInstance(odb::dbInst* inst, bool left) const
{
  const Rect inst_rect = inst->getBBox()->getBox();
  DbuX x_dbu = left ? DbuX{inst_rect.xMin() - 1} : DbuX{inst_rect.xMax() + 1};
  x_dbu -= core_.xMin();
  GridX x = grid_->gridX(x_dbu);

  GridY y = grid_->gridSnapDownY(DbuY{inst_rect.yMin() - core_.yMin()});

  Pixel* pixel = grid_->gridPixel(x, y);

  odb::dbInst* adjacent_inst = nullptr;

  // do not return macros, endcaps and tapcells
  if (pixel != nullptr && pixel->cell && pixel->cell->getDbInst()->isCore()) {
    adjacent_inst = pixel->cell->getDbInst();
  }

  return adjacent_inst;
}

std::vector<dbInst*> Opendp::getAdjacentInstancesCluster(dbInst* inst) const
{
  const bool left = true;
  const bool right = false;
  std::vector<odb::dbInst*> adj_inst_cluster;

  odb::dbInst* left_inst = getAdjacentInstance(inst, left);
  while (left_inst != nullptr) {
    adj_inst_cluster.push_back(left_inst);
    // the right instance can be ignored, since it was added in the line above
    left_inst = getAdjacentInstance(left_inst, left);
  }

  std::reverse(adj_inst_cluster.begin(), adj_inst_cluster.end());
  adj_inst_cluster.push_back(inst);

  odb::dbInst* right_inst = getAdjacentInstance(inst, right);
  while (right_inst != nullptr) {
    adj_inst_cluster.push_back(right_inst);
    // the left instance can be ignored, since it was added in the line above
    right_inst = getAdjacentInstance(right_inst, right);
  }

  return adj_inst_cluster;
}

/* static */
bool Opendp::isInside(const Rect& cell, const Rect& box)
{
  return cell.xMin() >= box.xMin() && cell.xMax() <= box.xMax()
         && cell.yMin() >= box.yMin() && cell.yMax() <= box.yMax();
}

bool Opendp::checkOverlap(const Rect& cell, const Rect& box)
{
  return box.xMin() < cell.xMax() && box.xMax() > cell.xMin()
         && box.yMin() < cell.yMax() && box.yMax() > cell.yMin();
}

void Opendp::groupInitPixels()
{
  for (GridX x{0}; x < grid_->getRowSiteCount(); x++) {
    for (GridY y{0}; y < grid_->getRowCount(); y++) {
      Pixel* pixel = grid_->gridPixel(x, y);
      pixel->util = 0.0;
    }
  }
  for (auto& group : arch_->getRegions()) {
    if (group->getCells().empty()) {
      if (group->getId() != 0) {
        logger_->warn(
            DPL, 42, "No cells found in group {}. ", group->getName());
      }
      continue;
    }
    const DbuX site_width = grid_->getSiteWidth();
    for (const DbuRect rect : group->getRects()) {
      debugPrint(logger_,
                 DPL,
                 "detailed",
                 1,
                 "Group {} region [x{} y{}] [x{} y{}]",
                 group->getName(),
                 rect.xl.v,
                 rect.yl.v,
                 rect.xh.v,
                 rect.yh.v);
      const GridRect grid_rect{grid_->gridWithin(rect)};

      for (GridY k{grid_rect.ylo}; k < grid_rect.yhi; k++) {
        for (GridX l{grid_rect.xlo}; l < grid_rect.xhi; l++) {
          Pixel* pixel = grid_->gridPixel(l, k);
          pixel->util += 1.0;
        }
        if (rect.xl % site_width != 0) {
          Pixel* pixel = grid_->gridPixel(grid_rect.xlo, k);
          pixel->util
              -= (rect.xl % site_width).v / static_cast<double>(site_width.v);
        }
        if (rect.xh % site_width != 0) {
          Pixel* pixel = grid_->gridPixel(grid_rect.xhi - 1, k);
          pixel->util -= ((site_width - rect.xh) % site_width).v
                         / static_cast<double>(site_width.v);
        }
      }
    }
    for (const DbuRect rect : group->getRects()) {
      const GridRect grid_rect{grid_->gridWithin(rect)};

      for (GridY k{grid_rect.ylo}; k < grid_rect.yhi; k++) {
        for (GridX l{grid_rect.xlo}; l < grid_rect.xhi; l++) {
          // Assign group to each pixel.
          Pixel* pixel = grid_->gridPixel(l, k);
          if (pixel->util == 1.0) {
            pixel->group = group;
            pixel->is_valid = true;
            pixel->util = 1.0;
          } else if (pixel->util > 0.0 && pixel->util < 1.0) {
            pixel->cell = dummy_cell_.get();
            pixel->util = 0.0;
            pixel->is_valid = false;
          }
        }
      }
    }
  }
}

int divRound(const int dividend, const int divisor)
{
  return round(static_cast<double>(dividend) / divisor);
}

int divCeil(const int dividend, const int divisor)
{
  return ceil(static_cast<double>(dividend) / divisor);
}

int divFloor(const int dividend, const int divisor)
{
  return dividend / divisor;
}

}  // namespace dpl
