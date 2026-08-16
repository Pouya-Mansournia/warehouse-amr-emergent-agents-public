import json

import pytest

from experiment_manager.run_paths import (
    _find_repo_root,
    finalize_duration,
    make_run_dir,
    write_metadata,
)


def test_make_run_dir_does_not_collide(monkeypatch, tmp_path):
    import experiment_manager.run_paths as run_paths

    monkeypatch.setattr(run_paths, "EXPERIMENTS_ROOT", tmp_path)

    first = make_run_dir("centralized_baseline")
    second = make_run_dir("centralized_baseline")

    assert first != second
    assert first.exists() and second.exists()
    assert (first / "rosbag").is_dir()
    assert (first / "plots").is_dir()


def test_write_and_finalize_metadata(monkeypatch, tmp_path):
    import experiment_manager.run_paths as run_paths

    monkeypatch.setattr(run_paths, "EXPERIMENTS_ROOT", tmp_path)
    run_dir = make_run_dir("centralized_baseline")

    write_metadata(
        run_dir,
        mode="centralized_baseline",
        num_robots=3,
        seed=42,
        ros_distro="jazzy",
        gazebo_version="harmonic",
    )
    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["robot_count"] == 3
    assert metadata["random_seed"] == 42
    assert metadata["duration_sec"] is None
    assert (run_dir / "seed.txt").read_text() == "42"

    finalize_duration(run_dir, 12.5)
    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["duration_sec"] == 12.5


def test_write_metadata_defaults_to_wall_time_source(monkeypatch, tmp_path):
    import experiment_manager.run_paths as run_paths

    monkeypatch.setattr(run_paths, "EXPERIMENTS_ROOT", tmp_path)
    run_dir = make_run_dir("centralized_baseline")

    write_metadata(
        run_dir, mode="centralized_baseline", num_robots=2, seed=1,
        ros_distro="jazzy", gazebo_version="harmonic",
    )
    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["time_source"] == "wall"
    assert metadata["simulation_duration_sec"] is None
    assert metadata["realtime_factor"] is None


def test_finalize_duration_computes_realtime_factor_when_simulation_duration_given(
    monkeypatch, tmp_path
):
    import experiment_manager.run_paths as run_paths

    monkeypatch.setattr(run_paths, "EXPERIMENTS_ROOT", tmp_path)
    run_dir = make_run_dir("centralized_baseline")
    write_metadata(
        run_dir, mode="centralized_baseline", num_robots=2, seed=1,
        ros_distro="jazzy", gazebo_version="harmonic", time_source="simulation",
    )

    finalize_duration(run_dir, 300.0, simulation_duration_sec=30.0)

    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["duration_sec"] == 300.0
    assert metadata["simulation_duration_sec"] == 30.0
    assert metadata["realtime_factor"] == 0.1


def test_finalize_duration_without_simulation_duration_leaves_realtime_factor_none(
    monkeypatch, tmp_path
):
    import experiment_manager.run_paths as run_paths

    monkeypatch.setattr(run_paths, "EXPERIMENTS_ROOT", tmp_path)
    run_dir = make_run_dir("centralized_baseline")
    write_metadata(
        run_dir, mode="centralized_baseline", num_robots=2, seed=1,
        ros_distro="jazzy", gazebo_version="harmonic",
    )

    finalize_duration(run_dir, 12.5)

    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["duration_sec"] == 12.5
    assert metadata["simulation_duration_sec"] is None
    assert metadata["realtime_factor"] is None


def test_find_repo_root_locates_git_dir_regardless_of_nesting_depth(tmp_path):
    repo = tmp_path / "repo"
    (repo / ".git").mkdir(parents=True)
    # Mirrors a plain (non-symlink) colcon install: the file ends up several extra
    # directories deep under install/.../site-packages/... rather than under src/ -
    # this must still resolve to the actual repo root, not that install subtree.
    deeply_nested = repo / "install" / "pkg" / "lib" / "python3.12" / "site-packages" / "pkg"
    deeply_nested.mkdir(parents=True)
    fake_file = deeply_nested / "run_paths.py"
    fake_file.touch()

    assert _find_repo_root(fake_file) == repo


def test_find_repo_root_raises_when_no_git_dir_found(tmp_path):
    orphan = tmp_path / "no_repo_here" / "deep" / "file.py"
    orphan.parent.mkdir(parents=True)
    orphan.touch()

    with pytest.raises(RuntimeError):
        _find_repo_root(orphan)
