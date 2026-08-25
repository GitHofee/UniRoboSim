# UniRoboSim demos

English | [简体中文](README.zh-CN.md)

This library teaches the three public integration levels separately. Choose the
path that matches the software you are building; you do not need to learn Adapter
internals to write an application.

| Path | Intended reader | Use it when you need to |
|---|---|---|
| [EasyAPI](easyapi/README.md) | Simulation application developer | Create a scene, send commands, and read observations with the shortest public API |
| [Runtime API](runtime_api/README.md) | Upper-layer framework or service developer | Own backend selection, lifecycle, typed World specifications, commands, and observations |
| [Adapter SPI](adapter_spi/README.md) | Simulator integration maintainer | Implement and publish a new backend without changing Core |

## Before you start

Core supports Python 3.11 and 3.12. From a source checkout, either install it once:

```bash
python -m pip install -e .
```

or run Core-only cases with the source tree explicitly visible:

```bash
PYTHONPATH=src python demo/easyapi/01_first_simulation/main.py
```

EasyAPI cases 01–09 and all Runtime API cases use the deterministic FakeProvider.
They verify portable contracts without installing a native simulator. Fake motion
and camera patterns are not physics- or rendering-fidelity evidence.

EasyAPI case 10 has also passed a visible Isaac Lab 3.0 / Isaac Sim 6.0.1 run with
Adapter 0.10.1 and a 1280×720 backend camera. The public MuJoCo and PyBullet lines
still require Core 0.9, so their Core 0.10 executions remain pending.

The Adapter SPI path includes its own installable teaching distribution. Follow its
[one-time setup](adapter_spi/README.md#one-time-setup) before running that path.

## Verification labels

- **Core contract:** executable without a simulator SDK; checks API behavior and
  deterministic assertions.
- **Installed distribution:** checks package metadata and Python entry-point
  discovery in the current environment.
- **Clean wheel:** builds artifacts and installs them into an isolated temporary
  environment.
- **Native pending:** source is ready for an installed Adapter, but no compatible
  native Core/Adapter pair was exercised by this release gate.

Every numbered case contains executable Python, an English README, and a Chinese
README. Commands are written from the repository root, programs are bounded, and
assertions make a silent wrong result fail with a non-zero exit status.
