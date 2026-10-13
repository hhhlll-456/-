from vision_core import calib_store


def test_load_missing_returns_defaults():
    c = calib_store.load_calib(path="/nonexistent/x.json")
    assert c["K"][0][0] == 1460.76
    assert c["dist"] == [0.0] * 5
    assert c["boundary_side_mm"] == 25.0
    assert c["vehicle_side_mm"] == 30.0


def test_save_then_load_roundtrip(tmp_path):
    p = str(tmp_path / "c.json")
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0}}, path=p)
    assert calib_store.load_calib(path=p)["bias"]["dy_mm"] == 55.0