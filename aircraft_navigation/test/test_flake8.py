from pathlib import Path

from ament_flake8.main import main


def test_flake8():
    package_root = Path(__file__).parents[1]
    assert main(argv=[str(package_root / 'aircraft_navigation')]) == 0
