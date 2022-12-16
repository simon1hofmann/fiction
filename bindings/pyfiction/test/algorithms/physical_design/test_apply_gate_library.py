from fiction.pyfiction import *
import unittest
import os

dir_path = os.path.dirname(os.path.realpath(__file__))


class TestApplyGateLibrary(unittest.TestCase):

    def test_apply_qca_one_library(self):
        network = read_logic_network(dir_path + "/../../resources/mux21.v")
        layout = orthogonal(network)

        cell_layout = apply_qca_one_library(layout)

    def test_apply_bestagon_library(self):
        network = read_logic_network(dir_path + "/../../resources/mux21.v")

        params = exact_params()
        params.scheme = "ROW"
        params.crossings = True
        params.border_io = True

        layout = exact_hexagonal(network, params)

        cell_layout = apply_bestagon_library(layout)