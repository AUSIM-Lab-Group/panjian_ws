#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace semantic_guard {

// Teacher-v1 F09 finite candidate set. q=0 is beta_pre_guard and q=Q is
// the explicit zero candidate; no continuous maximisation is claimed.
inline double teacherBacktrackingCandidate(double beta_pre_guard,
                                           double kappa,
                                           std::size_t q,
                                           std::size_t max_backtracks) {
    if (!std::isfinite(beta_pre_guard) || beta_pre_guard < 0.0 ||
        !std::isfinite(kappa) || kappa <= 0.0 || kappa > 1.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (q >= max_backtracks) return 0.0;
    return beta_pre_guard * std::pow(kappa, static_cast<double>(q));
}

// The teacher draft leaves the scalar risk-ranking formula open. During the
// pilot beta_tilde is the explicit recorded proxy and ID breaks ties.
inline std::vector<std::size_t> teacherRiskOrder(
    const std::vector<unsigned int>& obstacle_ids,
    const std::vector<double>& beta_tilde) {
    std::vector<std::size_t> order(obstacle_ids.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t lhs,
                                                     std::size_t rhs) {
        const double l = lhs < beta_tilde.size() && std::isfinite(beta_tilde[lhs])
                             ? beta_tilde[lhs] : -std::numeric_limits<double>::infinity();
        const double r = rhs < beta_tilde.size() && std::isfinite(beta_tilde[rhs])
                             ? beta_tilde[rhs] : -std::numeric_limits<double>::infinity();
        if (l != r) return l > r;
        return obstacle_ids[lhs] < obstacle_ids[rhs];
    });
    return order;
}

}  // namespace semantic_guard
