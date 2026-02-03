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
    // Clip macros to core area first (leaving halo towards the boundary)
    clipInstBoundingBox(macro, halo_width);

    // Then snap locations to the nearest valid one
    snapper.setMacro(macro);
    snapper.snapMacro();
    

  }
}

// Clip the inst to the inside of the core area by its bounding box
void Legalizer::clipInstBoundingBox(dbInst* inst, int halo) {
  // Get core dimensions for clipping
  odb::Rect core = db_->getChip()->getBlock()->getCoreArea();

  // Get current position before clipping
  odb::Point old_pos = inst->getOrigin();
  odb::Point current_pos = old_pos;
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

  // Actual clipping, add/subtract halo once to keep it towards the edge
  if (inst_bbox.xMin() < core.xMin()) {
    current_pos.setX(core.xMin() + halo);
  } else if (inst_bbox.xMax() > core.xMax()) {
    current_pos.setX(core.xMax() - raw_inst_bbox.dx() - halo);
  }
  if (inst_bbox.yMin() < core.yMin()) {
    current_pos.setY(core.yMin() + halo);
  } else if (inst_bbox.yMax() > core.yMax()) {
    current_pos.setY(core.yMax() - raw_inst_bbox.dy() - halo);
  }

  // Set new position
  inst->setOrigin(current_pos.getX(), current_pos.getY());

  // Warn if clipping moved the macro significantly (potential grid alignment issue)
  int movement_x = std::abs(current_pos.getX() - old_pos.getX());
  int movement_y = std::abs(current_pos.getY() - old_pos.getY());
  int total_movement = movement_x + movement_y;

  if (total_movement > 0) {
    logger_->warn(MPL,
                  56,
                  "Macro {} was moved {} DBU during clipping after snapping "
                  "(dx={}, dy={}). This may cause pins to go off routing grid.",
                  inst->getName(),
                  total_movement,
                  movement_x,
                  movement_y);
  }

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
  
  // Save force vectors and velocity vectors for every macro
  std::map<dbInst*, Vector2D> forces = std::map<dbInst*, Vector2D>();
  std::map<dbInst*, Vector2D> velocities = std::map<dbInst*, Vector2D>();
  
  // Init velocities to 0 here, forces every iteration
  for (auto& macro : macros_) {
    velocities[macro] = Vector2D(0, 0);
  }

  // Get core area for boundary forces
  odb::Rect core = db_->getChip()->getBlock()->getCoreArea();
  odb::Point center_die = core.center();
  
  // Calculate max distance from center to boundary for later boundary forces
  double max_distance = sqrt(pow(core.dx() / 2.0, 2) + pow(core.dy() / 2.0, 2));

  // Save total overlap for every iteration
  int64_t total_overlap = 0;
  
  // Counter for consecutive iterations with zero overlap
  int zero_overlap_count = 0;

  for (int iteration = 0; iteration < max_iter; iteration++) {
    // Reset force accumulation at the beginning of every iteration
    for (auto& macro : macros_) {
      forces[macro] = Vector2D(0, 0);
    }
    // Same for total overlap area
    total_overlap = 0;

    // Having one block per force isn't best for performance
    // but it kees the code cleaner imo and we have enough performance

    // Calculate overlap forces by looping over all macros i
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

        // Check for rect intersection
        if (rect_i.intersects(rect_j)) {

          // Get direction from center i to j 
          Vector2D direction = Vector2D(rect_i.center(), rect_j.center());
          
          // If magnitude is very small just use a small horizontal direction as fallback
          if (direction.getMagnitude() < 1e-6) {
            direction = Vector2D(1.0, 0.0);
          } else {
            // else normalize
            direction.normalize();

            // Add small perpendicular pertubabtion for stacked macros along one axis
            // Scale 1 / origin multiplier to overcome it pulling them back
            float pertubation_strength;
            if (origin_multiplier > 0.0) {
              pertubation_strength = 1.0 / origin_multiplier;
            } else {
              pertubation_strength = 5.0; // magic number alert
            }

            if (std::abs(direction.getX()) < 1e-6) {
              direction = Vector2D(pertubation_strength, direction.getY());
            } 
            else if (std::abs(direction.getY()) < 1e-6) {
              direction = Vector2D(direction.getX(), pertubation_strength);
            }
            direction.normalize();
          }

          // To scale the force, take min of overlap x and y distance
          odb::Rect overlap_rect = rect_i.intersect(rect_j);
          float overlap_dist = std::min(overlap_rect.dx(), overlap_rect.dy());

          // (while we're here add overlap_rects area to accumulator)
          total_overlap += overlap_rect.area();

          // Force vector is now direction * this distance
          Vector2D overlap_vector = direction * overlap_dist * overlap_multiplier;

          // TODO: remove this but it might come in handy again..
          // debugPrint(logger_, MPL, "macro", 1,
          //   "macro_i name: {} / macro_j name: {} / macro_i origin: {} / macro_j origin: {} / overlap_rect xMin, xMax, yMin, yMax: {}, {}, {}, {} / overlap_dist: {} / direction x,y: {}, {} / direction post normalize x, y: {}, {} / final magnitude: {}",
          //   macro_i->getName(),
          //   macro_j->getName(),
          //   macro_i->getOrigin(),
          //   macro_j->getOrigin(),
          //   overlap_rect.xMin(),
          //   overlap_rect.xMax(),
          //   overlap_rect.yMin(),
          //   overlap_rect.yMax(),
          //   overlap_dist,
          //   Vector2D(rect_i.center(), rect_j.center()).getX(),
          //   Vector2D(rect_i.center(), rect_j.center()).getY(),
          //   direction.getX(),
          //   direction.getY(),
          //   overlap_vector.getMagnitude()
          // );

          // Add this force to the forces map for macros i and j
          forces[macro_i] = forces[macro_i] - overlap_vector;
          forces[macro_j] = forces[macro_j] + overlap_vector;
        }
      }
    } // end macro pair loop

    // Spring forces to original position
    if (origin_multiplier > 0.0) {
      for (auto macro : macros_) {
        odb::Point current_position = macro->getOrigin();
        odb::Point original_position = original_coordinates[macro];

        // Compute the vector again
        Vector2D direction = Vector2D(current_position, original_position);

        if (direction.getMagnitude() > 1e-6) {
          forces[macro] = forces[macro] + (direction * origin_multiplier);
        }
      }
    }

    // Boundary push force
    if (boundary_multiplier > 0.0) {
      for (auto macro : macros_) {
        odb::Point current_center = macro->getBBox()->getBox().center();

        // Get direction from center of the die to the current center
        Vector2D direction = Vector2D(center_die, current_center);

        // Scale this force so that its strongest at the center and falls off to the edge
        float scalar = pow((max_distance - direction.getMagnitude()) / max_distance, 3);

        // Finally, apply to the forces with the multiplier
        forces[macro] = forces[macro] + (direction * scalar * boundary_multiplier);
      }
    }

    logger_->info(MPL, 38, "Iteration {}: total overlap = {}", iteration, total_overlap);

    ////////////////
    // Force computation complete, now update velocities based on these forces
    ////////////////

    for (auto macro : macros_) {
      // Update velocities
      velocities[macro] = velocities[macro] * (1.0 - damping_factor) + forces[macro];

      debugPrint(logger_, MPL, "macro", 1, "Velocities for macro {}: ({}/{})", 
        macro->getName(), velocities[macro].getX(), velocities[macro].getY());

      // Update position by applying velocities in x and y component
      auto old_pos = macro->getOrigin();
      macro->setOrigin(
        old_pos.getX() + velocities[macro].getX(),
        old_pos.getY() + velocities[macro].getY()
      );

      // Additionally clip macros to core area
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
  }

  // Before we return, maybe macros are already well placed, just with overlap from the halo bloat..
  //  so check if all macros overlap with 95% of the halo width
  if (total_overlap > 0) {
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