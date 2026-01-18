#pragma once

#include <vector>

#include "odb/db.h"

namespace mpl {

using std::vector;

using odb::dbInst;
using odb::dbDatabase;

using utl::Logger;

class Legalizer {
  public:
    Legalizer() = default;
    ~Legalizer() = default;

    void init(utl::Logger* logger, odb::dbDatabase* db);

    void snapMacros(vector<dbInst*> macros, int halo_width);
    void fixMacroPlacement(float overlap_multiplier, float origin_multiplier, float boundary_multiplier, float damping_factor, int halo_width_raw, int max_iter);

  private:
    void clipInstBoundingBox(dbInst* inst, int halo = 0);

    Logger* logger_ = nullptr;
    dbDatabase* db_ = nullptr;

    vector<dbInst*> macros_;
    vector<dbInst*> std_cells_;
};

} // namespace mpl