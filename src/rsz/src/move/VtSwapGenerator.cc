// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "VtSwapGenerator.hh"

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "VtSwapCandidate.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/Scene.hh"
#include "sta/Transition.hh"

namespace rsz {

VtSwapGenerator::VtSwapGenerator(
    const GeneratorContext& context,
    std::unordered_set<sta::Instance*>* not_swappable)
    : MoveGenerator(context), not_swappable_(not_swappable)
{
}

bool VtSwapGenerator::isApplicable(const Target& target) const
{
  // Base checks both path-driver and instance views via requiredViews().
  // Instance-only targets additionally need the not_swappable_ tracker.
  return MoveGenerator::isApplicable(target)
         && (target.canBePathDriver() || not_swappable_ != nullptr);
}

std::vector<std::unique_ptr<MoveCandidate>> VtSwapGenerator::generate(
    const Target& target)
{
  std::vector<std::unique_ptr<MoveCandidate>> candidates;
  sta::Pin* drvr_pin = nullptr;
  sta::Instance* inst = nullptr;
  sta::LibertyCell* curr_cell = nullptr;
  sta::LibertyCell* best_cell = nullptr;
  if (!selectBestCell(target, drvr_pin, inst, curr_cell, best_cell)) {
    return candidates;
  }

  candidates.push_back(std::make_unique<VtSwapCandidate>(
      resizer_, target, drvr_pin, inst, curr_cell, best_cell));
  return candidates;
}

bool VtSwapGenerator::selectBestCell(const Target& target,
                                     sta::Pin*& drvr_pin,
                                     sta::Instance*& inst,
                                     sta::LibertyCell*& curr_cell,
                                     sta::LibertyCell*& best_cell) const
{
  if (target.canBePathDriver()) {
    return selectPathBestCell(target, drvr_pin, inst, curr_cell, best_cell);
  }
  return selectInstanceBestCell(target, inst, curr_cell, best_cell);
}

bool VtSwapGenerator::selectPathBestCell(const Target& target,
                                         sta::Pin*& drvr_pin,
                                         sta::Instance*& inst,
                                         sta::LibertyCell*& curr_cell,
                                         sta::LibertyCell*& best_cell) const
{
  drvr_pin = target.resolvedPin(resizer_);
  if (drvr_pin == nullptr) {
    return false;
  }

  inst = target.inst(resizer_);

  return inst != nullptr && resizer_.vtCategoryCount() >= 2
         && !resizer_.dontTouch(inst) && resizer_.isLogicStdCell(inst)
         && resolvePathCurrentCell(drvr_pin, curr_cell)
         && selectBestEquivCell(curr_cell, best_cell);
}

bool VtSwapGenerator::selectInstanceBestCell(const Target& target,
                                             sta::Instance*& inst,
                                             sta::LibertyCell*& curr_cell,
                                             sta::LibertyCell*& best_cell) const
{
  inst = target.inst(resizer_);
  curr_cell = resizer_.network()->libertyCell(inst);
  return curr_cell != nullptr
         && resizer_.checkAndMarkVTSwappable(inst, *not_swappable_, best_cell)
         && best_cell != nullptr;
}

bool VtSwapGenerator::resolvePathCurrentCell(sta::Pin* drvr_pin,
                                             sta::LibertyCell*& curr_cell) const
{
  sta::LibertyPort* drvr_port = resizer_.network()->libertyPort(drvr_pin);
  curr_cell = drvr_port != nullptr ? drvr_port->libertyCell() : nullptr;
  return curr_cell != nullptr
         && resizer_.dbNetwork()->staToDb(curr_cell) != nullptr;
}

bool VtSwapGenerator::selectBestEquivCell(sta::LibertyCell* curr_cell,
                                          sta::LibertyCell*& best_cell) const
{
  best_cell = nullptr;
  const sta::LibertyCellSeq equiv_cells = resizer_.getVTEquivCells(curr_cell);
  auto it = std::ranges::find(equiv_cells, curr_cell);
  if (it == equiv_cells.end()) {
    return false;
  }

  const sta::MinMax* max = sta::MinMax::max();
  sta::LibertyCell* running_best = curr_cell;
  for (++it; it != equiv_cells.end(); ++it) {
    sta::LibertyCell* cand_cell = *it;
    bool weakens_drive = false;
    for (sta::Scene* scene : resizer_.sta()->scenes()) {
      const int lib_ap = scene->libertyIndex(max);
      sta::LibertyCell* best_corner = running_best->sceneCell(lib_ap);
      sta::LibertyCell* cand_corner = cand_cell->sceneCell(lib_ap);
      if (best_corner == nullptr || cand_corner == nullptr) {
        continue;
      }
      sta::LibertyCellPortIterator port_iter(best_corner);
      while (port_iter.hasNext()) {
        sta::LibertyPort* best_port = port_iter.next();
        if (!best_port->direction()->isAnyOutput()) {
          continue;
        }
        sta::LibertyPort* cand_port
            = cand_corner->findLibertyPort(best_port->name());
        if (cand_port == nullptr
            || cand_port->driveResistance(sta::RiseFall::rise(), max)
                   > best_port->driveResistance(sta::RiseFall::rise(), max)
            || cand_port->driveResistance(sta::RiseFall::fall(), max)
                   > best_port->driveResistance(sta::RiseFall::fall(), max)) {
          weakens_drive = true;
          break;
        }
      }
      if (weakens_drive) {
        break;
      }
    }
    if (!weakens_drive) {
      running_best = cand_cell;
    }
  }

  if (running_best != curr_cell) {
    best_cell = running_best;
  }
  return best_cell != nullptr;
}

}  // namespace rsz
