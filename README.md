# Live-Autotune-ORBSLAM3

ORB-SLAM3 fork with live, per-keyframe local bundle-adjustment covariance autotuning: every
local BA call re-estimates edge information (measurement covariance) online per ORB-pyramid
octave, instead of using fixed weights. Pass `-` for the autotune config to run vanilla
ORB-SLAM3 with none of this.

## Running it

`stereo_kitti` (and the other `Examples/*` binaries) takes a fixed argument list:

```
./stereo_kitti <vocabulary> <settings.yaml> <sequence_path> <autotune_config.yaml | -> \
               <loop_closing:0|1> <viewer:0|1> <playback_speed> <trajectory_output_dir>
```

- `autotune_config.yaml | -`: a tuning config (below), or `-` to disable autotuning.
- `trajectory_output_dir`: absolute path; `CameraTrajectory.txt` and `autotune_clamp_stats.csv`
  are written there.

```bash
./bin/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI04-12.yaml \
    /data/KITTI/07 configs/example_autotune.yaml 0 1 1.0 /tmp/run_00
```

## Autotune config

Only the fields below are read (`AutotuneConfig` in `include/Autotune.h` has the defaults for
anything you omit) — this is the full, real schema, not a superset:

```yaml
%YAML:1.0
---
autotune:
  outer_iterations: 4       # BA passes per keyframe
  inner_iterations: 25      # optimizer iterations per pass
  tune_iterations: 10       # covariance re-estimation rounds per pass
  pre_iterations: 2         # warm-up passes before tuning starts
  post_iterations: 2        # passes after tuning stops (weights held fixed)
  prior_strength: 0.1       # weight of the prior vs. the empirical estimate
  use_wishart_prior: 1      # seed the per-octave covariance with a Wishart prior
  use_identity_prior: 0     # if 1, skip the empirical estimate; identity throughout
  use_k_as_denom: 0
  diagonal_constraint: 0
  huber_delta_mono: 2.4477540726
  huber_delta_stereo: 2.7954784921
  verbose: 1

solver:
  min_eig_cov: 1.0e-03
  max_eig_cov: 100.0
  octave_bands:              # group ORB pyramid octaves that share one tuned covariance
    - octaves: [0]
    - octaves: [1]
    - octaves: [2]
    - octaves: [3]
    - octaves: [4]
    - octaves: [5]
    - octaves: [6]
    - octaves: [7]
```

## Forming an experiment

One run = one `(sequence, autotune_config)` pair, invoked as above. A sweep — multiple
sequences, multiple tuning configs, Monte-Carlo repeats, ATE evaluation — is just this binary
called once per combination; that orchestration lives outside this repo (in VSLAM-LAB), not
here. This binary only ever needs the 8 arguments above to run standalone.
