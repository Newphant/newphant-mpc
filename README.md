# newphant-mpc

Learned-dynamics, delay-aware residual CEM-MPC for position-controlled manipulators.
A GRU models the closed-loop position-actuator dynamics; CEM searches bounded joint
reference residuals around a continuous DLS-IK nominal, with asynchronous planning,
command projection and delay-aware execution.

## Layout

| Path | Contents |
|---|---|
| `control/` | CEM-MPC controller, costs, constraints, IK, delay/packet handling |
| `models/` | MuJoCo robot models, learned-dynamics training and evaluation |
| `runtime/` | simulation and real-robot backend boundary |
| `tools/` | experiment, calibration and analysis entry points |
| `configs/` | robot and experiment specifications |

## Install

```bash
python -m pip install -r requirements.txt
```

## Run

```bash
python tools/run_cem_mpc.py --help
python tools/generate_task_reference.py --help
```

Run commands from the repository root; entry points add the root and `models/`
to `sys.path` themselves.
