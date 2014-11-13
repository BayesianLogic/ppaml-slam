from ppaml_car.data import ALL_DATASETS
from ppaml_car.data import Dataset

# import nose


def test_all_datasets_readable():
    for name in ALL_DATASETS:
        Dataset.read(name)
