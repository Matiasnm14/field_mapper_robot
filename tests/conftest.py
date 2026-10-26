from pathlib import Path

import pytest

from field_mapper.config import load_config


@pytest.fixture
def config():
    root = Path(__file__).resolve().parents[1]
    return load_config(root / "configs/simulation_dsv60.json")
