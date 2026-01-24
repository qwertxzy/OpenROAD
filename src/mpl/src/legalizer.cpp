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

  for (auto inst : all_insts) {
    if (inst->isBlock()) {
      macros_.push_back(inst);
    }
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
void Legalizer::fixMacroPlacement(
  float overlap_multiplier, 
  float origin_multiplier, 
  float boundary_multiplier, 
  float damping_factor, 
  int halo_width_raw, 
  int max_iter,
  int consecutive_zero_iters
) {
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
  
  // Initialize velocity vectors
  std::map<dbInst*, Vector2D> velocities = std::map<dbInst*, Vector2D>();
  for (auto& macro : macros_) {
    velocities[macro] = Vector2D(0, 0);
  }

  // Get core area for boundary forces
  odb::Rect core = db_->getChip()->getBlock()->getCoreArea();
  odb::Point center_die = core.center();
  
  // Calculate max distance from center to boundary
  double max_distance = sqrt(pow(core.dx() / 2.0, 2) + pow(core.dy() / 2.0, 2));

  // Save total overlap for this iteration
  int64_t total_overlap = 0;
  
  // Counter for consecutive iterations with zero overlap
  int zero_overlap_count = 0;

  for (int iteration = 0; iteration < max_iter; iteration++) {
    total_overlap = 0;

    // Initialize forces with a spring to each macro's original position
    for (auto& macro : macros_) {
      odb::Point original_position = original_coordinates[macro];
      odb::Point macro_pos = macro->getOrigin();
      Vector2D spring_vector = Vector2D(macro_pos, original_position);

      // Add multiplied spring force to forces map
      forces[macro] = spring_vector * origin_multiplier;
    }

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

          Vector2D force_vector = direction * overlap_dist;

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

      // Also add boundary push forces
      odb::Point current_pos = macro_i->getOrigin();
      odb::Rect macro_bbox = macro_i->getBBox()->getBox();
      odb::Point macro_center = macro_bbox.center();
      
      // Direction from die center to macro center
      Vector2D direction = Vector2D(center_die, macro_center);
      double distance = direction.getMagnitude();
      
      if (distance > 1e-6) {
        direction.normalize();
        
        // Boundary factor smallest at boundary, larger near center
        // Cubic falloff as these got very large
        double boundary_factor = pow((max_distance - distance) / max_distance, 3);
        
        Vector2D boundary_force = direction * (boundary_multiplier * boundary_factor);
        forces[macro_i] = forces[macro_i] + boundary_force;
      }
    }

    logger_->info(MPL, 38, "Iteration {}: total overlap = {}", iteration, total_overlap);

    // Apply velocity-based damping and update positions
    for (auto& macro : macros_) {
      Vector2D force_vector = forces[macro];
      
      // Update velocity with damping: v = (1 - damping) * v + overlap_multiplier * F
      velocities[macro] = velocities[macro] * (1.0 - damping_factor) + force_vector * overlap_multiplier;

      odb::Point current_pos = macro->getOrigin();
      
      // Move macro by velocity
      current_pos.addX(velocities[macro].getX());
      current_pos.addY(velocities[macro].getY());
      
      // Set new position and clip it
      macro->setOrigin(current_pos.getX(), current_pos.getY());
      clipInstBoundingBox(macro, halo_width);
    }

    // Check if overlap is zero and update counter
    if (total_overlap == 0) {
      zero_overlap_count++;
      
      // Check if we've had enough consecutive zero overlap iterations
      if (zero_overlap_count >= consecutive_zero_iters) {
        logger_->info(MPL, 55, "Resolved all overlaps in {} iterations ({} consecutive zero-overlap iterations)", 
                      iteration + 1, zero_overlap_count);

        // Snap all macros to manufacturing grid
        snapMacros(macros_, halo_width);

        break;
      }
    } else {
      zero_overlap_count = 0;
    }

    // Clear forces for next iteration
    forces.clear();
  }

  // Before we return, maybe macros are already well placed, just with overlap from the halo bloat..
  //  so check if all macros overlap with 95% of the halo width
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