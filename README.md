# Live-Autotune-ORBSLAM3

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


----

## Running it via VSLAM-LAB

This repo is meant to sit at `VSLAM-LAB/Baselines/ORB-SLAM3-DEV/` inside a VSLAM-LAB checkout --
VSLAM-LAB builds it and drives the CLI above for you; you don't call `stereo_kitti` directly.

1. **`pixi.toml`**'s `orbslam3-dev` environment maps its `execute-stereo` task straight onto the
   CLI above:
   ```toml
   [feature.orbslam3-dev.tasks]
   execute-stereo = { cmd = "python ../extra-files/orbslam3_upstream/vslamlab_orbslam3_upstream_stereo.py",
                       cwd = "Baselines/ORB-SLAM3-DEV" }
   ```
2. **An experiment yaml** (`VSLAM-LAB/configs/exp_vslamlab.yaml`) selects this baseline via
   `Module: orbslam3-dev`; its `Parameters` become the `autotune_config`/`loop_closing`/`viewer`/
   `playback_speed` args:
   ```yaml
   exp_k0_prior:
     Config: /workspace/shared/config_kitti_567.yaml   # which sequences, e.g. KITTI 05/06/07
     NumRuns: 10                                       # Monte-Carlo repeats per sequence
     Parameters:
       mode: stereo
       autotune_config: /workspace/shared/experiment_manual_identity_prior_k0.yaml
       viewer: 1
       playback_speed: "0.5"
     Module: orbslam3-dev
   ```
3. **Run the sweep**:
   ```bash
   cd VSLAM-LAB && pixi run vslamlab configs/exp_vslamlab.yaml --overwrite
   ```
   This loops every (sequence x repeat), builds the CLI above once per run, then evaluates and
   compares results -- see VSLAM-LAB's own docs for `Config`/results-folder details.
