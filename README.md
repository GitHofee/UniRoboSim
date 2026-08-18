# UniRoboSim

UniRoboSim is a backend-neutral contract and runtime layer for robot simulators. It gives applications
one explicit lifecycle, capability model, world schema, data convention, and failure model while
keeping simulator SDKs behind adapters.

This repository is at **M1 / `v0alpha2`**. The implemented scope is deliberately small:

- immutable portable values using SI units, right-handed Z-up frames, XYZW quaternions, and
  batch-first arrays;
- versioned capability declaration and strict negotiation;
- immutable world specifications with deterministic fingerprints;
- provider protocols and registry with no simulator imports;
- surface and volume deformable topology/state/point-command contracts;
- fixed-count particle-fluid state and point-command contracts;
- a deterministic Fake Reference Backend for contract and lifecycle tests.

The fake backend uses independent point-mass reference rules and is not a soft-matter physics
simulator. Isaac Lab, Isaac Sim, MuJoCo, FEM/PBD/SPH, collision, rendering, sensors, debug
visualization, configuration compilation, and MCP are not implemented in M1.

## Development environment

The tested environment matches Isaac Sim 6.x and Isaac Lab 3.0:

```bash
conda env create -f environment.yml
conda activate unirobosim-dev
python -m pip install --no-build-isolation -e .
make verify
```

The package runtime itself has no third-party dependencies. The initial compatibility promise is
Python `>=3.12,<3.13`; additional versions will only be claimed after CI coverage exists.

## Contract-first example

```python
from unirobosim import EntityKind, EntityPath, EntitySpec, WorldSpec
from unirobosim.testing import FakeProvider

provider = FakeProvider()
session = provider.open()
world = session.build(
    WorldSpec(
        world_id="example",
        entities=(
            EntitySpec(
                path=EntityPath("/robot"),
                kind=EntityKind.ARTICULATION,
                joint_names=("joint_1",),
            ),
        ),
    )
)
world.reset()
world.step()
session.close()
```

Detailed architecture and verification evidence are maintained by the UniRoboSim project workspace under
`docs/unirobosim/`; this repository intentionally keeps no second documentation tree.
