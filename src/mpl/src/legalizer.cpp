#include "mpl2/legalizer.h"

#include "hier_rtlmp.h"

namespace mpl2 {

// Not using this because mpl2 defines its own rect..
// using odb::Rect;
using odb::Vector2D;
using odb::Point;
using odb::dbMasterType;

using mpl2::Snapper;

using utl::MPL;

void Legalizer::init(utl::Logger* logger, odb::dbDatabase* db) {
  logger_ = logger;
  db_ = db;

  auto block = db_->getChip()->getBlock();
  auto all_insts = block->getInsts();
  
  // Sort all insts into std_cells_ and macros_
  macros_.clear();
  std_cells_.clear();

  for (auto inst : all_insts) {
    if (isStdCell(inst)) {
      std_cells_.push_back(inst);
    } else {
      macros_.push_back(inst);
    }
  }
}

// Stolen from dpl/src/Objects.cpp
// TODO: Maybe check if the endcaps/tapcells can be skipped so this doesn't ruin floorplanning step
bool Legalizer::isStdCell(dbInst* inst) const {
  if (inst == nullptr) {
    return false;
  }
  dbMasterType type = inst->getMaster()->getType();
  // Use switch so if new types are added we get a compiler warning.
  switch (type.getValue()) {
    case dbMasterType::CORE:
    case dbMasterType::CORE_ANTENNACELL:
    case dbMasterType::CORE_FEEDTHRU:
    case dbMasterType::CORE_TIEHIGH:
    case dbMasterType::CORE_TIELOW:
    case dbMasterType::CORE_SPACER:
    case dbMasterType::CORE_WELLTAP:
    case dbMasterType::ENDCAP:
    case dbMasterType::ENDCAP_PRE:
    case dbMasterType::ENDCAP_POST:
    case dbMasterType::ENDCAP_TOPLEFT:
    case dbMasterType::ENDCAP_TOPRIGHT:
    case dbMasterType::ENDCAP_BOTTOMLEFT:
    case dbMasterType::ENDCAP_BOTTOMRIGHT:
    case dbMasterType::ENDCAP_LEF58_BOTTOMEDGE:
    case dbMasterType::ENDCAP_LEF58_TOPEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTEDGE:
    case dbMasterType::ENDCAP_LEF58_LEFTEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTBOTTOMEDGE:
    case dbMasterType::ENDCAP_LEF58_LEFTBOTTOMEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTTOPEDGE:
    case dbMasterType::ENDCAP_LEF58_LEFTTOPEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTBOTTOMCORNER:
    case dbMasterType::ENDCAP_LEF58_LEFTBOTTOMCORNER:
    case dbMasterType::ENDCAP_LEF58_RIGHTTOPCORNER:
    case dbMasterType::ENDCAP_LEF58_LEFTTOPCORNER:
      return true;
    case dbMasterType::BLOCK:
    case dbMasterType::BLOCK_BLACKBOX:
    case dbMasterType::BLOCK_SOFT:
      // These classes are completely ignored by the placer.
    case dbMasterType::COVER:
    case dbMasterType::COVER_BUMP:
    case dbMasterType::RING:
    case dbMasterType::PAD:
    case dbMasterType::PAD_AREAIO:
    case dbMasterType::PAD_INPUT:
    case dbMasterType::PAD_OUTPUT:
    case dbMasterType::PAD_INOUT:
    case dbMasterType::PAD_POWER:
    case dbMasterType::PAD_SPACER:
      return false;
  }
  // gcc warniing
  return false;
}

// Method to unplace std cells
void Legalizer::unplaceStdCells() {
  for (dbInst* cell : std_cells_) {
    cell->setPlacementStatus(odb::dbPlacementStatus::UNPLACED);
  }
}

// Snap all macros to the manufacturing grid & align pins where possible
void Legalizer::snapMacros(vector<dbInst*> macros)
{
  debugPrint(logger_, MPL, "macro", 1, "Snapping {} macros to manufacturing grid", macros.size());
  Snapper snapper(logger_);
  
  for (auto& macro : macros) {
    snapper.setMacro(macro);
    snapper.snapMacro();
    
    // Lock position
    macro->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
  }
}

// LEO: Method to fix slight overlaps between macros after GPL
// Plan:
//  - Every macro has a spring force to its original position
//  - Every macro has a repulsive force for each overlap
//  - Iterate until overlaps are resolved or max_iter is reached
void Legalizer::fixMacroPlacement(float overlap_multiplier, float origin_multiplier, float damping_factor, int halo_width_raw, int max_iter) {

  // Convert raw halo width to dbu
  int halo_width = halo_width_raw * db_->getTech()->getDbUnitsPerMicron();

  // Get a list of all macro's original coordinates
  std::map<dbInst*, Point> original_coordinates;
  for (auto macro : macros_) {
    original_coordinates[macro] = macro->getOrigin();

    // While we're at it, set all locked macros to just 'placed'
    if (macro->getPlacementStatus() == odb::dbPlacementStatus::LOCKED) {
      macro->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }
  }
  
  // Save force vectors for every macro
  std::map<dbInst*, Vector2D> forces = std::map<dbInst*, Vector2D>();

  // Initialize forces with a spring to each macro's original position
  for (auto& macro : macros_) {
    Point original_position = original_coordinates[macro];
    Point macro_pos = macro->getOrigin();
    Vector2D spring_vector = Vector2D(macro_pos, original_position);

    // Add multiplied spring force to forces map
    forces[macro] = spring_vector * origin_multiplier;
  }

  // Save total overlap for this iteration
  int64_t total_overlap = 0;

  for (int iteration = 0; iteration < max_iter; iteration++) {
    total_overlap = 0;
    // Loop over all macros i
    for (int i = 0; i < macros_.size(); i++) {
      dbInst* macro_i = macros_[i];
      odb::Rect rect_i;
      if (halo_width > 0) {
        macro_i->getBBox()->getBox().bloat(halo_width, rect_i);
      } else {
        rect_i = macro_i->getBBox()->getBox();
      }     

      // Loop over all other macros j
      for (int j = i + 1; j < macros_.size(); j++) {
        dbInst* macro_j = macros_[j];
        odb::Rect rect_j;
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
          odb::Rect overlap_rect = rect_i.intersect(rect_j);
          int overlap_dist = std::min(overlap_rect.dx(), overlap_rect.dy());
        
          debugPrint(logger_,
            MPL,
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
            MPL,
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

    logger_->info(MPL, 37, "Iteration {}: total overlap = {}", iteration, total_overlap);

    // Apply forces to all macros
    odb::Rect core = db_->getChip()->getBlock()->getCoreArea();
    for (auto& force : forces) {
      dbInst* macro = force.first;
      Vector2D force_vector = force.second;
      if (force_vector.getMagnitude() == 0) {
        continue;
      }
      
      debugPrint(logger_,
        MPL,
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
      logger_->info(MPL, 46, "Resolved all overlaps in {} iterations", iteration);

      // Snap all macros to manufacturing grid
      snapMacros(macros_);

      break;
    }

    // Clear forces for next iteration
    forces.clear();
  }

  // Before we return, maybe macros are already well placed, just with overlap from the halo bloat..
  //  so check if all macros overlap with 90% of the halo width
  if (total_overlap > 0 && halo_width > 0) {
    int adjusted_halo_width = round(halo_width * 0.95);
    bool mostly_overlap_free = true;

    // Loop over all macros i
    for (int i = 0; i < macros_.size(); i++) {
      if (!mostly_overlap_free) {
        break;
      }
      
      dbInst* macro_i = macros_[i];
      odb::Rect rect_i;
      macro_i->getBBox()->getBox().bloat(adjusted_halo_width, rect_i);
   
      // Loop over all other macros j
      for (int j = i + 1; j < macros_.size(); j++) {
        dbInst* macro_j = macros_[j];
        odb::Rect rect_j;
        macro_j->getBBox()->getBox().bloat(adjusted_halo_width, rect_j);

        // If any overlap, set flag
        if (rect_i.intersects(rect_j)) {
          mostly_overlap_free = false;
          break;
        }
      }
    }
    
    // If all maros are mostly overlap free, snap them anyway
    if (mostly_overlap_free) {
      logger_->info(MPL, 47, "95% of Halos were valid, snapped macros anyways");
      snapMacros(macros_);
    }
  }
}

}  // namespace mpl2