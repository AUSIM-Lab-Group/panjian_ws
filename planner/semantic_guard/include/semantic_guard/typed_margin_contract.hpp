#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace semantic_guard {

inline bool hasUniqueObstacleIds(const std::vector<uint32_t>& ids) {
  std::unordered_set<uint32_t> seen;
  seen.reserve(ids.size());
  for (const uint32_t id : ids) {
    if (!seen.insert(id).second) return false;
  }
  return true;
}

// Reorders values by explicit obstacle ID. Positional inheritance is forbidden:
// both ID sets must be identical and unique or the join fails closed.
inline bool reorderValuesByObstacleId(
    const std::vector<uint32_t>& source_ids,
    const std::vector<double>& source_values,
    const std::vector<uint32_t>& target_ids,
    std::vector<double>* reordered_values, std::string* reason = nullptr) {
  if (reordered_values == nullptr) {
    if (reason != nullptr) *reason = "null_output";
    return false;
  }
  reordered_values->clear();
  if (source_ids.size() != source_values.size()) {
    if (reason != nullptr) *reason = "source_count_mismatch";
    return false;
  }
  if (source_ids.size() != target_ids.size()) {
    if (reason != nullptr) *reason = "id_set_size_mismatch";
    return false;
  }
  if (!hasUniqueObstacleIds(source_ids) ||
      !hasUniqueObstacleIds(target_ids)) {
    if (reason != nullptr) *reason = "duplicate_id";
    return false;
  }

  std::unordered_map<uint32_t, double> value_by_id;
  value_by_id.reserve(source_ids.size());
  for (size_t index = 0; index < source_ids.size(); ++index) {
    value_by_id.emplace(source_ids[index], source_values[index]);
  }
  reordered_values->reserve(target_ids.size());
  for (const uint32_t id : target_ids) {
    const auto it = value_by_id.find(id);
    if (it == value_by_id.end()) {
      reordered_values->clear();
      if (reason != nullptr) *reason = "missing_or_extra_id";
      return false;
    }
    reordered_values->push_back(it->second);
  }
  if (reason != nullptr) *reason = "ok";
  return true;
}

inline void retainAcceptedMarginsForActiveIds(
    const std::vector<uint32_t>& active_ids,
    std::map<uint32_t, double>* accepted_by_id) {
  if (accepted_by_id == nullptr) return;
  const std::unordered_set<uint32_t> active(active_ids.begin(),
                                             active_ids.end());
  for (auto it = accepted_by_id->begin(); it != accepted_by_id->end();) {
    if (active.count(it->first) == 0) {
      it = accepted_by_id->erase(it);
    } else {
      ++it;
    }
  }
}

inline bool acceptedMarginsForIds(
    const std::vector<uint32_t>& ids,
    const std::map<uint32_t, double>& accepted_by_id,
    std::vector<double>* values) {
  if (values == nullptr) return false;
  values->clear();
  values->reserve(ids.size());
  for (const uint32_t id : ids) {
    const auto it = accepted_by_id.find(id);
    if (it == accepted_by_id.end()) {
      values->clear();
      return false;
    }
    values->push_back(it->second);
  }
  return true;
}

inline bool storeAcceptedMargins(
    const std::vector<uint32_t>& ids, const std::vector<double>& values,
    std::map<uint32_t, double>* accepted_by_id) {
  if (accepted_by_id == nullptr || ids.size() != values.size() ||
      !hasUniqueObstacleIds(ids)) {
    return false;
  }
  for (size_t index = 0; index < ids.size(); ++index) {
    (*accepted_by_id)[ids[index]] = values[index];
  }
  return true;
}

}  // namespace semantic_guard
