from pathlib import Path


def test_no_ground_truth_dependency():
    package_root = Path(__file__).parents[1]
    source_files = (package_root / 'aircraft_navigation').rglob('*.py')
    source = '\n'.join(path.read_text() for path in source_files)
    assert 'model_states' not in source
    assert 'gazebo/model' not in source
