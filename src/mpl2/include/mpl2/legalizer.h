#pragma once

#include <vector>

#include "odb/db.h"

namespace mpl2 {

using std::vector;

using odb::dbInst;
using odb::dbDatabase;

using utl::Logger;

class Legalizer {
  public:
    Legalizer() = default;
    ~Legalizer() = default;

    void init(utl::Logger* logger, odb::dbDatabase* db);

    void unplaceStdCells();
    void snapMacros(vector<dbInst*> macros, int halo_width);
    void fixMacroPlacement(float overlap_multiplier = 0.3, float origin_multiplier = 0.05, float damping_factor = 0.2, int halo_width = 0, int max_iter = 100);

  private:
    bool isStdCell(dbInst* inst) const;
    void clipInstBoundingBox(dbInst* inst, int halo = 0);

    Logger* logger_ = nullptr;
    dbDatabase* db_ = nullptr;

    vector<dbInst*> macros_;
    vector<dbInst*> std_cells_;
};

} // namespace mpl2