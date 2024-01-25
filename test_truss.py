import unittest
import sys
import os
import numpy as np

# your main script for truss solver
import main

class Test(unittest.TestCase):

    def setUp(self):
        path = os.path.dirname(__file__)
        self.TEST_KEYS = [f for f in os.listdir(path) if f.endswith('.txt')]
        self.TEST_CASES = {}
        for i in self.TEST_KEYS:
            if i == 'truss1.txt':
                self.TEST_CASES[i] = [np.array([[-1.00000000e+01],
                                                [ 1.41400000e+01],
                                                [-1.68251802e-15],
                                                [-5.30658738e-15],
                                                [-1.99969798e+01],
                                                [-9.99848989e+00],
                                                [ 1.51011402e-03],
                                                [ 1.99969798e+01]]), np.array([False, False, False, False, False])]
            elif i == 'truss2.txt':
                self.TEST_CASES[i] = [np.array([[-22500.        ],
                                                [ 37500.        ],
                                                [-22500.        ],
                                                [ 31819.80515339],
                                                [ -7500.        ],
                                                [-22500.        ]]), np.array([False, False, False])]

        return

    def test_function(self):
        """function to test the result of the truss solver
        """
        # test truss1 and test truss2
        for key, item in self.TEST_CASES.items():
            output = main.run(key, False)
            self.assertIsNone(np.testing.assert_allclose(output[0], item[0], rtol=1e-4, atol=0), "Solver returns incorrect unknown forces")
            self.assertIsNone(np.testing.assert_array_equal(output[1], item[1]), "Solver returns incorrect failure status of bars")
      
    def test_deterministic(self):
        """function to test if truss is deterministic
        """
        with self.assertRaises(ValueError) as cm: 
            # truss3.txt
            output = main.run('truss3.txt', True)

    def test_member_connection(self):
        """function to test if the member connects to non-declared nodes
        """
        with self.assertRaises(ValueError) as cm:
            # truss4.txt
            output = main.run('truss4.txt', True)

    def test_external_force(self):
        """function to test if the external force acts on the non-declared nodes
        """
        with self.assertRaises(ValueError) as cm:
            # truss5.txt
            output = main.run('truss5.txt', True)

    def test_member_configs(self):
        """function to test if the member's configs are specified (e.g young's modulus, yield strength, and dimensions)
        """
        with self.assertRaises(ValueError) as cm:
            # truss6.txt
            output = main.run('truss6.txt', True)

if __name__ == '__main__':
    unittest.main(argv=['first-arg-is-ignored'], exit=False)