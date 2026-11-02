import os
import sys

import pytest

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
SDF = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "worlds", "dice_mission.sdf"))


@pytest.fixture(scope="session")
def world():
    from mission_planner.planner import World
    return World(SDF)
