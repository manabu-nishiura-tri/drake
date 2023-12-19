import numpy as np
import scipy.sparse as sp
import unittest

import pydrake.planning as mut
from pydrake.common.test_utilities import numpy_compare
from pydrake.geometry.optimization import HPolyhedron, Hyperrectangle
from pydrake.common import RandomGenerator


class TestIrisFromCliqueCover(unittest.TestCase):
    def test_point_sampler_base_subclassable(self):
        class UniformHPolyhedronSamplerManual(mut.PointSamplerBase):
            def __init__(self, set, generator):
                mut.PointSamplerBase.__init__(self)
                self.set = set
                self.generator = generator

            def DoSamplePoints(self, num_points):
                ret = np.zeros((self.set.ambient_dimension(), num_points))
                for i in range(num_points):
                    ret[:, i] = self.set.UniformSample(self.generator)
                return ret

        A = np.array([[-1, 0], [0, -1], [1, 1]])
        b = np.array([0, 0, 1])
        set = HPolyhedron(A, b)
        generator = RandomGenerator(0)
        sampler = UniformHPolyhedronSamplerManual(set, generator)
        numpy_compare.assert_equal(sampler.set.A(), A)
        numpy_compare.assert_equal(sampler.set.b(), b)

        num_points = 3
        self.assertEqual(sampler.SamplePoints(num_points).shape, (2, 3))

    def test_uniform_set_sampler(self):
        A = np.array([[-1, 0], [0, -1], [1, 1]])
        b = np.array([0, 0, 1])
        set = HPolyhedron(A,b)
        generator = RandomGenerator(0)
        mut.UniformHPolyhedronSampler(set=set)
        sampler = mut.UniformHPolyhedronSampler(set=set, generator=generator)
        numpy_compare.assert_equal(sampler.set.A(), A)
        numpy_compare.assert_equal(sampler.set.b(), b)

        num_points = 3
        self.assertEqual(sampler.SamplePoints(num_points).shape, (2, 3))

    #     lb = np.array([0, 0, 0])
    #     ub = np.array([1, 1, 1])
    #     set = Hyperrectangle(lb, ub)
    #     mut.UniformHyperrectangleSampler(set=set)
    #     sampler = mut.UniformHyperrectangleSampler(set=set, generator=generator)
    #     numpy_compare.assert_equal(sampler.set.lb(), lb)
    #     numpy_compare.assert_equal(sampler.set.ub(), ub)
    #     self.assertEqual(sampler.SamplePoints(num_points).shape, (3, 3))
    #
    # def test_rejection_sampler(self):
    #     lb = np.array([0, 0, 0])
    #     ub = np.array([1, 1, 1])
    #     set = Hyperrectangle(lb, ub)
    #     generator = RandomGenerator(0)
    #     base_sampler = mut.UniformSetSampler(set=set, generator=generator)
    #     sampler = mut.RejectionSampler(sampler=base_sampler,
    #                                    rejection_fun=lambda x: np.any(x > 0.5))
    #     self.assertEqual(sampler.SamplePoints(7).shape, (3, 7))
    #     self.assertFalse(True)
    #     print("HERE")
        # numpy_compare.assert_equal(sampler.set.lb(), lb)
        # numpy_compare.assert_equal(sampler.set.ub(), ub)
        # self.assertEqual(sampler.SamplePoints(num_points).shape, (3, 3))



