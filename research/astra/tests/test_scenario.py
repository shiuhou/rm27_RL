import numpy as np
from research.astra.scenario import Scenario


def test_mass_and_supply_draws_are_independent():
    s = Scenario(1000, 5)
    assert abs(np.corrcoef(s.mass_error, s.performance)[0, 1]) < .1
