from pathlib import Path


def test_data(*args):
    data = Path(__file__).parent / "test_data"
    return str(data.joinpath(*args))
