# Contributing

Thanks for your interest in this project. The codebase is small, but
contributions, feature requests and bug reports are welcome.

## Development setup

```bash
git clone https://github.com/Janga786/kuka-kr6-kinematics.git
cd kuka-kr6-kinematics

python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate

make dev                              # installs the package + dev deps
make test                             # runs the full pytest suite
```

## Layout

| Path                  | Contents                                                        |
| --------------------- | --------------------------------------------------------------- |
| `kr6_kinematics/`     | The library — DH/FK, IK, Jacobian, trajectories, viz, URDF gen. |
| `scripts/*.py`        | Topical driver scripts; produce figures and animations.         |
| `tests/`              | `pytest` suite that verifies analytic results numerically.      |
| `figures/`, `animations/` | Reproducible artefacts emitted by the driver scripts.       |
| `urdf/`               | Auto-generated URDF.                                            |

## Coding standards

- **Format & lint:** `make lint` (ruff). Line length 100.
- **Type hints:** encouraged on public APIs, optional internally. The
  package ships `py.typed` and is checked with mypy in `make typecheck`.
- **Tests:** every change to `kr6_kinematics/` must keep the existing
  39-test suite green and ideally add a new test that exercises the
  change.
- **Docstrings:** reStructuredText / NumPy style. Lead with one-line
  summary, then a short prose description, then `Parameters`,
  `Returns`, `Notes`.
- **Math notation:** prefer LaTeX inside docstrings for any equation
  beyond the trivial; the README is rendered with MathJax/KaTeX.

## Submitting changes

1. Open an issue describing the bug or feature you'd like to address.
2. Fork the repo, create a topic branch from `main`.
3. Make your change with tests.
4. Run `make lint` and `make test` locally.
5. Open a pull request with a short rationale.

By submitting a pull request you agree that your contribution will be
released under the project's MIT license.
