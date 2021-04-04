from typing import List
from helpers import Map, load_map
from solution import shortest_path
from tests import test


###################################
# Tests
###################################
test(shortest_path)


###################################
# Demo
###################################
map_40: Map = load_map('map-40.pickle')

path: List[int] = shortest_path(map_40, 8, 24)
print(path)