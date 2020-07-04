import unittest
from ..src.occ_grid_lib import OccMap

map = OccMap(5, 5)

class TestStringMethods(unittest.TestCase):
    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FOO')


if __name__ == '__main__':
    unittest.main()

