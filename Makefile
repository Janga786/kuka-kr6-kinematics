# =============================================================================
#  KUKA KR 6 R900 sixx — kinematics suite — top-level Makefile
# =============================================================================

PY        ?= python3
PIP       ?= $(PY) -m pip
PYTEST    ?= $(PY) -m pytest

.PHONY: help install dev test lint typecheck figures animations all clean

help:  ## show this help
	@awk 'BEGIN{FS=":.*##"; printf "Targets:\n"} /^[a-zA-Z_-]+:.*?##/ {printf "  \033[1;36m%-14s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install:  ## install the package (editable) and runtime dependencies
	$(PIP) install -e .

dev:  ## install the package + dev/test dependencies
	$(PIP) install -e .[dev]

test:  ## run the full pytest suite
	$(PYTEST) -q tests/

lint:  ## ruff lint
	$(PY) -m ruff check kr6_kinematics scripts tests

typecheck:  ## mypy type-check
	$(PY) -m mypy kr6_kinematics

figures:  ## regenerate every figure (no animations)
	$(PY) scripts/01_robot_id.py
	$(PY) scripts/02_dh_frames.py
	$(PY) scripts/03_forward_kinematics.py
	$(PY) -c "import scripts._paths; from scripts.04_joint_trajectories import quintic_profile_figure, joints_panel_figure; quintic_profile_figure(); joints_panel_figure()" 2>/dev/null \
	  || $(PY) scripts/04_joint_trajectories.py
	$(PY) scripts/05_jacobian_singularities.py
	$(PY) scripts/06_inverse_kinematics.py

animations:  ## regenerate every animated GIF
	$(PY) scripts/04_joint_trajectories.py
	$(PY) scripts/05_jacobian_singularities.py
	$(PY) scripts/07_task_space_trajectory.py
	$(PY) scripts/08_urdf_visualization.py

all:  ## regenerate every figure, animation and the URDF
	$(PY) scripts/run_all.py

clean:  ## remove build / cache artefacts (keeps figures & animations)
	rm -rf .pytest_cache .ruff_cache .mypy_cache build dist *.egg-info
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
