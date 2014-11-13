import nose
import numpy as np

from numutil import norm_log_pdf
from numutil import norm_log_pdf_id_cov
from numutil import norm_pdf


def test_norm_pdf():
    """
    norm_pdf result matches with scipy 0.14.0 implementation
    """
    # Note: Neither my code nor theirs check that cov is symmetric, and they
    # will give different results if cov is asymmetric.
    x = np.array([4., 5.])
    mean = np.array([3., 2.])
    cov = np.array([[3., 2.], [2., 4.]])
    ref_p = 0.017161310176083477
    # nose.tools.assert_almost_equals(
    #     scipy.stats.multivariate_normal.pdf(x, mean, cov), ref_p, 10)
    nose.tools.assert_almost_equals(
        norm_pdf(x, mean, cov), ref_p, 10)


def test_norm_log_pdf_sanity():
    """
    norm_log_pdf result matches with scipy 0.14.0 implementation
    """
    # Note: Neither my code nor theirs check that cov is symmetric, and they
    # will give different results if cov is asymmetric.
    x = np.array([4., 5.])
    mean = np.array([3., 2.])
    cov = np.array([[3., 2.], [2., 4.]])
    ref_logp = -4.0650978372492634
    # nose.tools.assert_almost_equals(
    #     scipy.stats.multivariate_normal.logpdf(x, mean, cov), ref_logp, 10)
    nose.tools.assert_almost_equals(
        norm_log_pdf(x, mean, cov), ref_logp, 10)


def test_norm_log_pdf_tinydet():
    """
    norm_log_pdf works when determinant of cov doesn't fit in double precision
    """
    mean = np.zeros(400)
    cov = np.eye(400) * 0.001
    # Determinant is too small to fit in double precision.
    nose.tools.assert_almost_equals(np.linalg.det(cov), 0.0, 10)
    # norm_log_pdf works nonetheless.
    nose.tools.assert_almost_equals(
        norm_log_pdf(np.zeros(400), mean, cov), 1013.9756425145674, 10)
    nose.tools.assert_almost_equals(
        norm_log_pdf(np.ones(400) * 0.2, mean, cov), -6986.024357485432, 10)


def test_norm_log_pdf_id_cov_sanity():
    """
    norm_log_pdf_id_cov result matches norm_log_pdf result
    """
    x = np.array([4., 5.])
    mean = np.array([3., 2.])
    cov_scale = 5.0
    nose.tools.assert_almost_equals(
        norm_log_pdf(x, mean, cov_scale * np.eye(2)),
        norm_log_pdf_id_cov(x, mean, cov_scale), 10)
