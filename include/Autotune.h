#ifndef ORB_SLAM3_AUTOTUNE_H
#define ORB_SLAM3_AUTOTUNE_H

#include <cmath>
#include <string>
#include <vector>

#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/optimizable_graph.h"

namespace ORB_SLAM3
{

struct AutotuneConfig
{
    int outer_iterations = 2;
    int inner_iterations = 10;
    int tune_iterations = 1;
    int pre_iterations = 0;
    int post_iterations = 0;
    bool use_wishart_prior = true;
    bool use_k_as_denom = false;
    bool diagonal_constraint = true;
    double min_eig_cov = 1.0e-3;
    double max_eig_cov = 100.0;
    double prior_strength = 0.1;
    bool use_identity_prior = false;  // skip the empirical mean-covariance step; identity for both initial edge information and the prior
    std::vector<std::vector<int>> octave_bands = {{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}};

    // Drives both the local BA g2o optimizer's own per-iteration printout (the same
    // thing g2o::optimize() prints when setVerbose(true)) and cov_auto_tune's own
    // verbose/verbose_diagnostics output.
    bool verbose = false;

    double huber_delta_mono = std::sqrt(5.991);
    double huber_delta_stereo = std::sqrt(7.815);
};


AutotuneConfig LoadAutotuneConfig(const std::string& path);

// Dumps every recorded {outer_iteration, tune_iteration, pct_low, pct_high} row -- one per
// Autotuner::tune() call made by RunLiveOctaveAutotune, across the whole process lifetime --
// to `path` as CSV. Intended to be called once, at shutdown.
void SaveAutotuneClampStatsCSV(const std::string& path);

// vpEdgesMono/vpEdgesStereo need only be visual reprojection edges -- pass the generic
// base pointer so this works whether the caller's concrete edge types are the plain
// (non-inertial) EdgeSE3ProjectXYZ/EdgeStereoSE3ProjectXYZ or the inertial-BA EdgeMono/
// EdgeStereo. Edges outside these two lists (e.g. EdgeInertial/EdgeGyroRW/EdgeAccRW in
// LocalInertialBA) are simply never registered with the autotuner and are left untouched
// -- cov_auto_tune's allow_unregistered_edges lets those pass through unassigned instead
// of throwing.
void RunLiveOctaveAutotune(
    g2o::SparseOptimizer& optimizer,
    const std::vector<g2o::OptimizableGraph::Edge*>& vpEdgesMono,
    const std::vector<int>& vnOctaveMono,
    const std::vector<g2o::OptimizableGraph::Edge*>& vpEdgesStereo,
    const std::vector<int>& vnOctaveStereo,
    const AutotuneConfig& config = AutotuneConfig(),
    bool* pbStopFlag = nullptr);

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_AUTOTUNE_H
