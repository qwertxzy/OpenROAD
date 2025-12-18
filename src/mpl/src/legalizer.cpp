#include "mpl/legalizer.h"

#include "hier_rtlmp.h"

namespace mpl {

// Not using this because mpl defines its own rect..
// using odb::Rect;
using odb::Vector2D;
using odb::Point;
using odb::dbMasterType;

using mpl::Snapper;

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
void Legalizer::snapMacros(vector<dbInst*> macros, int halo_width)
{
  debugPrint(logger_, MPL, "macro", 1, "Snapping {} macros to manufacturing grid", macros.size());
  Snapper snapper(logger_);
  
  for (auto& macro : macros) {
    snapper.setMacro(macro);
    snapper.snapMacro();
    
    // Snapper can have moved the macro out of the core again
    // .. so we clip it one more time
    clipInstBoundingBox(macro, halo_width);
  }
}

// Clip the inst to the inside of the core area by its bounding box
void Legalizer::clipInstBoundingBox(dbInst* inst, int halo) {
  // Get core dimensions for clipping  
  odb::Rect core = db_->getChip()->getBlock()->getCoreArea();

  // Get current position
  odb::Point current_pos = inst->getOrigin();
  odb::Rect raw_inst_bbox = inst->getBBox()->getBox();

  // Bloat int bbox so we can leave the halo towards the core edge too
  odb::Rect inst_bbox;
  raw_inst_bbox.bloat(halo, inst_bbox);

  debugPrint(logger_,
    MPL,
    "macro",
    2,
    "BBox of macro {} before clipping: ({}, {}), ({}, {})",
    inst->getName(),
    inst_bbox.xMin(),
    inst_bbox.yMin(),
    inst_bbox.xMax(),
    inst_bbox.yMax()
  );

  // Actual clipping
  if (inst_bbox.xMin() < core.xMin()) {
    current_pos.setX(core.xMin());
  } else if (inst_bbox.xMax() > core.xMax()) {
    current_pos.setX(core.xMax() - inst_bbox.dx());
  }

  if (inst_bbox.yMin() < core.yMin()) {
    current_pos.setY(core.yMin());
  } else if (inst_bbox.yMax() > core.yMax()) {
    current_pos.setY(core.yMax() - inst_bbox.dy());
  }

  // Set new position
  inst->setOrigin(current_pos.getX(), current_pos.getY());

  // update bbox for the debug print
  inst_bbox = inst->getBBox()->getBox();
  debugPrint(logger_,
    MPL,
    "macro",
    2,
    "BBox of macro {} after clipping: ({}, {}), ({}, {})",
    inst->getName(),
    inst_bbox.xMin(),
    inst_bbox.yMin(),
    inst_bbox.xMax(),
    inst_bbox.yMax()
  );
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
  std::map<dbInst*, odb::Point> original_coordinates;
  for (dbInst* macro : macros_) {
    // While we're at it, set all locked macros to just 'placed'
    if (macro->getPlacementStatus() == odb::dbPlacementStatus::LOCKED) {
      macro->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }
    // Clip the macro position initially
    clipInstBoundingBox(macro, halo_width);

    // ..and save the new position as the original coordinate
    original_coordinates[macro] = macro->getOrigin();
  }
  
  // Save force vectors for every macro
  std::map<dbInst*, Vector2D> forces = std::map<dbInst*, Vector2D>();

  // Initialize forces with a spring to each macro's original position
  for (auto& macro : macros_) {
    odb::Point original_position = original_coordinates[macro];
    odb::Point macro_pos = macro->getOrigin();
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

    logger_->info(MPL, 38, "Iteration {}: total overlap = {}", iteration, total_overlap);

    // Apply forces to all macros
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

      odb::Point current_pos = macro->getOrigin();
      
      // Move macro by force vector
      current_pos.addX(force_vector.getX());
      current_pos.addY(force_vector.getY());
      
      // Set new position and clip it
      macro->setOrigin(current_pos.getX(), current_pos.getY());
      clipInstBoundingBox(macro, halo_width);
    }

    // If total overlap area was 0, break out of the loop
    if (total_overlap == 0) {
      logger_->info(MPL, 49, "Resolved all overlaps in {} iterations", iteration);

      // Snap all macros to manufacturing grid
      snapMacros(macros_, halo_width);

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
      logger_->info(MPL, 54, "95% of Halos were valid, snapped macros anyways");
      snapMacros(macros_, halo_width);
    } else{
      logger_->error(MPL, 48, "Macro legaization could not resolve overlaps in the given number of iterations.");
      // Do not lock macros so we can run legalization again with different parameters
      return;
    }
  }

  // Lock positions
  for (dbInst* macro : macros_) {
    macro->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
  }
}

}  // namespace mpl