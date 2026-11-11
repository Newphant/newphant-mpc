# newphant-mpc

A learned-dynamics, delay-aware residual CEM-MPC controller for position-controlled
robot manipulators.

The controller does not search an unconstrained command trajectory. It tracks a
continuously generated inverse-kinematics nominal and searches only **bounded joint
reference residuals** around it. A GRU predicts the closed-loop position-actuator
response to candidate commands, and the cross-entropy method (CEM) picks the residual
sequence that minimises a joint-space cost. A separate execution layer projects every
candidate onto physical position/velocity/acceleration/braking limits, so a planner
failure always degrades to the nominal command instead of an unsafe one.

## Control loop

```text
task-space reference ──► continuous DLS IK ──► absolute joint nominal q_nom
                                                        │
state history ──► GRU rollout ◄── candidate residuals ──┤
                        │                                │
                        ▼                                ▼
                 CEM scoring  ──►  selected q_ref  ──► projection ──► actuators
                        ▲                                │
                        └──── timed plan packet ◄────────┘
```

Two timing models are supported:

* **synchronous** — replan every *N* steps, execute immediately (baseline).
* **virtual_asap** — deterministic logical delay; a plan becomes active a fixed number
  of steps after it was launched (reproducible ablations).
* **threaded_asap** — a CUDA planner thread runs asynchronously while a fixed-rate
  execution loop consumes timestamped plan packets, applies feedback correction and
  falls back to the nominal when a packet is late or missing (wall-clock behaviour).
* **virtual_smooth** — fixed-interval replanning with smoothed command hand-over.

## Repository layout

| Path | Contents |
|---|---|
| `control/` | CEM planner, rollout, cost functions, constraints, IK, delay protocol, packet handling, recovery, diagnostics |
| `control/tests/` | Unit and integration tests (33 test modules) |
| `models/` | MuJoCo robot descriptions and assets, GRU/MLP/Transformer dynamics, dataset and training code |
| `models/neural_dynamics/` | Dynamics models, normalisation, dataset loading, rollout utilities, MuJoCo environment wrapper |
| `models/scripts/` | Data collection, training, evaluation and diagnostics entry points |
| `models/robots/` | Per-robot MuJoCo models (`abb_irb2400`, `ur5e`, `so101`, `so101_fine`) |
| `runtime/` | Simulation / real-hardware boundary: backends, state history, executable command projection, safety checks |
| `tools/` | Experiment runners, reference generation, calibration, benchmarks, analysis |
| `configs/` | `RobotSpec` YAML files and experiment protocols |

## Requirements

* Python 3.10+
* `mujoco>=2.3.7`, `torch>=2.0`, `numpy`, `matplotlib`, `tqdm`, `PyYAML`
* A CUDA device for `threaded_asap` and compiled projection (the other modes run on CPU)
* Real SO101 hardware additionally needs the LeRobot SDK, listed in `requirements.txt`

```bash
python -m pip install -r requirements.txt
```

Run everything from the repository root. Entry points add the repository root and
`models/` to `sys.path` themselves, so no installation step is required.

## Quick start (simulation)

```bash
# 1. collect position-servo transitions
python models/scripts/collect_data.py \
  --robot_config configs/robots/abb_irb2400.yaml \
  --num_episodes 200 --episode_len 300 --num_envs 8 --action_std 0.5 \
  --save_path models/outputs/datasets/abb.npz

# 2. train the learned dynamics
python models/scripts/train_dynamics.py \
  --robot_config configs/robots/abb_irb2400.yaml \
  --data_path models/outputs/datasets/abb.npz \
  --model_type gru --history_len 16 --target_mode delta_dq --control_dt 0.01 \
  --epochs 100 --save_dir models/outputs/checkpoints

# 3. build a task-space reference (circle / ellipse / figure-8 / square)
python tools/generate_task_reference.py \
  --robot_config configs/robots/abb_irb2400.yaml \
  --shape figure8 --repeat_count 3 --save_dir outputs/references/figure8

# 4. run the controller
python tools/run_cem_mpc.py \
  --robot_config configs/robots/abb_irb2400.yaml \
  --checkpoint <CHECKPOINT>/best_model.pt \
  --normalizer <CHECKPOINT>/normalizer.pt \
  --model_type gru --history_len 16 \
  --reference_mode task --reference_file outputs/references/figure8/reference.npz \
  --horizon 20 --num_samples 128 --cem_iters 2 \
  --mpc_policy residual --residual_parameterization full \
  --multirate_mode threaded_asap --delay_protocol full \
  --anticipation_delay_steps 6 \
  --planner_projection on --planner_projection_backend compiled \
  --planner_projection_strategy two_stage \
  --cem_execute lowest_cost \
  --save_dir outputs/control/figure8
```

Direct-IK baseline (same reference, no learned residual):

```bash
python tools/run_cem_mpc.py \
  --robot_config configs/robots/abb_irb2400.yaml \
  --controller_mode ik_direct --reference_mode task \
  --reference_file outputs/references/figure8/reference.npz \
  --ik_command_projection raw --save_dir outputs/control/figure8_ik
```

## Command-line reference (main flags)

| Flag | Values / default | Meaning |
|---|---|---|
| `--controller_mode` | `mpc`, `ik_direct` | learned residual MPC or nominal-only baseline |
| `--model_type` | `mlp`, `gru`, `transformer` | dynamics architecture |
| `--history_len` | int (e.g. 16) | recurrent context length (tokens) |
| `--horizon` | `20` | planning horizon in control steps |
| `--reference_mode` | `hold`, `step`, `joint_sine`, `multi_joint_sine`, `waypoint`, `chirp`, `joint_file`, `task` | reference source |
| `--multirate_mode` | `synchronous`, `virtual_asap`, `virtual_smooth`, `threaded_asap` | timing model |
| `--delay_protocol` | `full`, `naive_delayed`, `no_future_alignment`, `no_reanchor`, `no_feedback` | delay-handling ablations |
| `--anticipation_delay_steps` | int | calibrated end-to-end activation delay |
| `--planner_projection` | `on`, `off` | kinematic/braking projection of planner candidates |
| `--planner_projection_backend` | `eager`, `compiled` | compiled backend needs CUDA |
| `--planner_projection_strategy` | `full`, `two_stage` | candidate-screening cost |
| `--cem_execute` | `mean`, `best`, `lowest_cost` | executed sample selection |
| `--uncertainty_mode` | `off`, `ensemble_monitor`, `ensemble_soft_gate`, `ensemble_gate` | model-disagreement monitor |
| `--residual_parameterization` | `full`, `linear_control_points` | residual basis |
| `--save_dir` | `outputs/control/cem_run` | run output directory |

## Residual semantics

* The action is always an **absolute joint reference**: `u_t = q_ref,t`.
* The planner optimises `r ∈ [-1, 1]`, mapped to per-joint limits from `residual_max`
  in the `RobotSpec` (for the 6-axis arm: `0.12, 0.10, 0.12, 0.15, 0.15, 0.20` rad).
* `zero residual` is exactly the current nominal, so the controller is a strict
  refinement of the IK baseline.
* The planner projection enforces joint, velocity, acceleration and braking limits on
  candidates; the execution layer re-applies a physical projection on the final command.
  Requested and executed residuals can differ after stateful projection and quantisation.
* Persistent tracking deterioration or residual saturation can trigger a recovery;
  hitting a command limit alone is diagnostic, not a recovery event.

## Learned dynamics

* One-step increment plus multi-step rollout loss, episode-disjoint train/validation split.
* Inputs are aligned `[q_t, dq_t, q_ref,t]` tokens; output target is `delta_dq`.
* `--target_mode`, `--control_dt`, `--history_len` must match between training and control.
* Datasets store `states`, `actions`, `next_states`, `episode_ids`, `q_ref`,
  `delta_q_ref`, actuator and gravity torques; a manifest beside each dataset records the
  collection configuration and is checked against the selected `RobotSpec`.

## Robots

| Robot | Model | Notes |
|---|---|---|
| ABB IRB 2400 | `models/ABB_IRB2400.xml` + `models/abb_irb2400_assets/` | primary MuJoCo evaluation target, 6 joints |
| UR5e | `models/robots/ur5e/` | independent model and calibration, 6 joints |
| SO101 | `models/robots/so101/`, `models/robots/so101_fine/` | 5-DOF printed arm, simulated and real-hardware backends |

Each `configs/robots/*.yaml` is a complete `RobotSpec`: joint/actuator order, TCP site,
home configuration, control period, gravity compensation, command limits, residual
bounds, servo scale and data-collection ranges. Always pass the full spec — an XML file
alone does not define the control contract.

## Tools

| Script | Purpose |
|---|---|
| `tools/generate_task_reference.py` | build and validate a task-space reference bundle |
| `tools/validate_ik.py` | re-validate a saved reference independently |
| `tools/run_cem_mpc.py` | one closed-loop MuJoCo rollout (MPC or direct IK) |
| `tools/run_cem_budget_sweep.py` | sample/iteration budget sweeps |
| `tools/benchmark_cem_projection.py` | planner projection latency/accuracy |
| `tools/benchmark_candidate_ranking.py` | candidate-ranking diagnostics |
| `tools/collect_mpc_data.py` | convert MPC rollouts into training datasets |
| `tools/run_real_cem_mpc.py` | real-hardware rollout with the SO101 backend |
| `tools/run_real_direct_control.py` | real-hardware nominal-only control |
| `tools/generate_so101_final_references.py` | real-hardware reference bundles |
| `tools/calibrate_*.py` | delay, mapping, table-safety, uncertainty calibration |
| `tools/model_c/` | dataset, benchmark and evaluation workflow |
| `tools/robustness/` | payload, gain-mismatch, force-pulse and observation-noise evaluation |
| `tools/analyze_*.py` | post-run analysis of rollouts, candidate ranking and alignment |

Every MPC run writes `rollout.npz`, `rollout.csv`, tracking/control plots and
`run_summary.json`; task-space runs additionally write `task_tracking_summary.json`.
Record tracking error together with planner rate, deadline misses, late-packet drops,
packet age, projection activity and fallback duty cycle.

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q control/tests
```

CUDA-dependent tests skip automatically when no device is available.

## Scope and limitations

* MuJoCo models are research simulations; position, velocity, acceleration and safety
  limits in the specs are not hardware ratings.
* Real-hardware workflows require a startup/homing procedure, operator-supported
  shutdown, collision checking and an independent emergency-stop path.
* The learned model is robot-specific: checkpoints, normalisers, references and delay
  calibrations are not transferable between robots without re-training and re-calibration.

## License

MIT. See [LICENSE](LICENSE).
