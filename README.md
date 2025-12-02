# workshop_qut_mobile_robots

Workshop materials and assets for QUT mobile robots sessions. This repository combines Python code (to be added under `src/`) and Rhino design files (under `rhino/`) used for robotics geometry, simulation, and fabrication workflows.

## Versioning

Project version is tracked in the standalone `VERSION` file and synchronized with `pyproject.toml` (PEP 621). We follow [Semantic Versioning](https://semver.org/):

- Increment MAJOR for incompatible changes
- Increment MINOR for new backwards-compatible features
- Increment PATCH for backwards-compatible fixes

Current version: `0.1.0`

## Repository Layout

```
data/                Raw and reference data (e.g. tool geometry)
docker/              Docker-related configs/resources
rhino/               Rhino assets (.3dm) for robot/tool paths
src/                 Python modules & scripts (add here)
VERSION              Single source of truth for version number
pyproject.toml       Project metadata & packaging config
.gitignore           Ignore rules for dev artifacts
```

## Development Setup

```powershell
# Create & activate virtual environment (Windows PowerShell)
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install project (editable when packages emerge)
pip install -e .

# (Add dependencies later via:)
pip install <package>
```

## Updating the Version

1. Decide new semantic version (e.g. 0.2.0).  
2. Edit `VERSION` file.  
3. Update `version` in `pyproject.toml` to match.  
4. Commit with message: `chore: bump version to x.y.z`.  
5. Tag (optional): `git tag -a vX.Y.Z -m "Release vX.Y.Z"` then `git push --tags`.

## Release Notes Template

```
## vX.Y.Z - YYYY-MM-DD
### Added
- 
### Changed
- 
### Fixed
- 
```

## License / Usage

Internal workshop materials. If licensing changes, update `license` field in `pyproject.toml` and note here.

## Contributing

1. Create a feature branch: `git checkout -b feature/<short-name>`  
2. Commit focused changes.  
3. Open a Pull Request describing context & testing.  
4. Keep assets in `rhino/` clean; avoid committing large temporary or backup files.

---
Happy building! 🤖