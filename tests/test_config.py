import json

import pytest

from field_mapper.config import config_from_dict, load_config


def test_profile_roundtrip_and_units(config, tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(config.to_dict()), encoding="utf-8")
    assert load_config(path) == config
    assert config.geometry.radius_mm == 30
    assert config.geometry.step_mm == 5


@pytest.mark.parametrize("field, value", [
    ("radius_mm", 0), ("radius_mm", -30), ("radius_mm", float("nan")),
    ("step_mm", float("inf")), ("step_mm", "5"), ("step_mm", True), ("step_mm", 7),
    ("bore_radius_mm", 29), ("bore_length_mm", 59),
])
def test_invalid_geometry_in_profile(config, field, value):
    data = config.to_dict()
    data["geometry"][field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("section", [None, "geometry", "units"])
def test_unknown_and_missing_fields_fail(config, section):
    data = config.to_dict()
    target = data if section is None else data[section]
    key = next(iter(target))
    value = target.pop(key)
    with pytest.raises(ValueError, match="faltantes"):
        config_from_dict(data)
    target[key] = value
    target["typo"] = 1
    with pytest.raises(ValueError, match="desconocidos"):
        config_from_dict(data)


@pytest.mark.parametrize("field, value", [("schema_version", True), ("schema_version", 2), ("name", " "), ("mode", "hardware")])
def test_unsupported_profile_metadata(config, field, value):
    data = config.to_dict()
    data[field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("field, unit", [("distance", "m"), ("angle", "rad"), ("field", "G")])
def test_incorrect_units_fail(config, field, unit):
    data = config.to_dict()
    data["units"][field] = unit
    with pytest.raises(ValueError, match="unidades"):
        config_from_dict(data)


@pytest.mark.parametrize("raw", [
    '{"name":"first","name":"second"}',
    '{"geometry":{"step_mm":5,"step_mm":10}}',
    '{"step_mm":NaN}', '{"step_mm":Infinity}', '[]', 'null', '{bad json',
])
def test_malformed_or_ambiguous_json_fails(raw, tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(raw, encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(path)
