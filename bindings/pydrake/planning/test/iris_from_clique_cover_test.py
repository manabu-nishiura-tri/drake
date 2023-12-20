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
        set = HPolyhedron(A, b)
        generator = RandomGenerator(0)
        mut.UniformHPolyhedronSampler(set=set)
        sampler = mut.UniformHPolyhedronSampler(set=set, generator=generator)
        numpy_compare.assert_equal(sampler.Set().A(), A)
        numpy_compare.assert_equal(sampler.Set().b(), b)

        num_points = 3
        self.assertEqual(sampler.SamplePoints(num_points).shape, (2, 3))

        lb = np.array([0, 0, 0])
        ub = np.array([1, 1, 1])
        set = Hyperrectangle(lb, ub)
        mut.UniformHyperrectangleSampler(set=set)
        sampler = mut.UniformHyperrectangleSampler(set=set,
                                                   generator=generator)
        numpy_compare.assert_equal(sampler.Set().lb(), lb)
        numpy_compare.assert_equal(sampler.Set().ub(), ub)
        self.assertEqual(sampler.SamplePoints(num_points).shape, (3, 3))

    def test_rejection_sampler(self):
        lb = np.array([0, 0, 0])
        ub = np.array([1, 1, 1])
        set = Hyperrectangle(lb, ub)
        generator = RandomGenerator(0)
        base_sampler = mut.UniformHyperrectangleSampler(set=set,
                                                        generator=generator)
        sampler = mut.RejectionSampler(sampler=base_sampler,
                                       rejection_fun=lambda x: np.any(x > 0.5))
        num_points = 7
        self.assertEqual(sampler.SamplePoints(num_points=num_points).shape,
                         (3, num_points))

    def test_coverage_checker_base_subclassable(self):
        class BadCoverageChecker(mut.CoverageCheckerBase):
            def __init__(self):
                mut.CoverageCheckerBase.__init__(self)

            def DoCheckCoverage(self, current_sets):
                return any([c.IsEmpty() for c in current_sets])


        checker = BadCoverageChecker()
        sets = [Hyperrectangle(np.array([0, 0]), np.array([1, 1])),
                HPolyhedron(np.array([[-1, 0], [0, -1], [1, 1]]),
                            np.array([0, 0, 1]))]
        self.assertFalse(checker.CheckCoverage(sets=sets))
        # A trivially empty set. This test is here to ensure that
        # DoCheckCoverage is actually being accessed.
        sets.append(HPolyhedron(np.array([[1],[-1]]), np.array([[-1],[-1]])))
        self.assertTrue(checker.CheckCoverage(sets=sets))

    def test_bernoulli_coverage_checker(self):
        domain = Hyperrectangle(np.array([0, 0]), np.array([1, 1]))
        alpha = 0.5
        num_points_per_check = 10
        point_sampler = mut.UniformHyperrectangleSampler(
            set=domain, generator=RandomGenerator(0))
        num_threads = 3
        point_in_set_tol = 1e-10

        checker = mut.CoverageCheckerViaBernoulliTest(alpha=alpha,
                                      num_points_per_check=num_points_per_check,
                                      sampler=point_sampler,
                                      num_threads=num_threads,
                                      point_in_set_tol=point_in_set_tol)
        self.assertEqual(checker.get_alpha(), alpha)
        checker.set_alpha(alpha=0.25)
        self.assertEqual(checker.get_alpha(), 0.25)

        self.assertEqual(checker.get_num_points_per_check(),
                         num_points_per_check)
        checker.set_num_points_per_check(num_points_per_check=5)
        self.assertEqual(checker.get_num_points_per_check(), 5)

        self.assertEqual(checker.get_num_threads(), num_threads)
        checker.set_num_threads(num_threads=2)
        self.assertEqual(checker.get_num_threads(), 2)

        self.assertEqual(checker.get_point_in_set_tol(), point_in_set_tol)
        checker.set_point_in_set_tol(point_in_set_tol=1e-8)
        self.assertEqual(checker.get_point_in_set_tol(), 1e-8)

        # We have 100% coverage since the sets is the domain. For numerical
        # reasons we just ensure the sampled coverage is over 90%.
        sets = [domain]
        self.assertGreaterEqual(checker.GetSampledCoverageFraction(sets=sets), 0.9)
        self.assertTrue(checker.CheckCoverage(sets=sets))


        default_checker = mut.CoverageCheckerViaBernoulliTest(
            alpha=alpha, num_points_per_check=num_points_per_check,
            sampler=point_sampler
        )
        self.assertEqual(default_checker.get_num_threads(), -1)
        self.assertEqual(default_checker.get_point_in_set_tol(), 1e-8)