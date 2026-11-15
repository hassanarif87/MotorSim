# motorsim

Models for simplified DC and PMSM motors.

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then
create the project environment (including notebook and test dependencies):

```bash
uv sync --locked --python 3.12
```

Slycot requires a C/Fortran compiler, CMake, and BLAS/LAPACK development
libraries when a compatible wheel is unavailable.

Run tests from the repository root:

```bash
uv run pytest
```

For notebooks, select `.venv/bin/python` as the Python kernel in your editor.
Start with `motorsim/pmsm_current_control.ipynb` for the connected current loop.

Use `uv add <package>` to add dependencies and `uv add --dev <package>` for
development tools. Commit both `pyproject.toml` and `uv.lock` after changes.
