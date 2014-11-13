import numpy as np


def norm_pdf(x, mu, sigma):
    """
    PDF of multivariate normal distribution.

    (scipy.stats.multivariate_normal only exists in scipy 0.14.0+)
    """
    k = len(x)
    norm_const = 1.0 / np.sqrt(((2 * np.pi) ** k) * np.linalg.det(sigma))
    sigma_inv = np.linalg.inv(sigma)
    return norm_const * np.exp(-0.5 * (x - mu).dot(sigma_inv).dot(x - mu))


def norm_log_pdf_fixed_sigma(sigma):
    """
    Return a function f(x, mu) that evaluates the norm_log_pdf for this sigma.

    Use this when you need to evaluate the pdf many times for some fixed sigma.
    """
    k = sigma.shape[0]
    _, logdet = np.linalg.slogdet(sigma)
    log_norm_const = -0.5 * (k * np.log(2 * np.pi) + logdet)
    sigma_inv = np.linalg.inv(sigma)

    def func(x, mu):
        return log_norm_const - 0.5 * (x - mu).dot(sigma_inv).dot(x - mu)

    return func


def norm_log_pdf(x, mu, sigma):
    """
    Log PDF of multivariate normal distribution.

    (Equivalent to scipy.stats.multivariate_normal.pdf, which doesn't exist in
    scipy < 0.14.)
    """
    return norm_log_pdf_fixed_sigma(sigma)(x, mu)


def norm_log_pdf_id_cov(x, mu, cov_scale):
    """
    Log PDF of MVG with scaled-identity covariance.

    (scipy.stats.multivariate_normal.pdf and norm_log_pdf both construct and
    invert the full covariance matrix, which is wasteful.)
    """
    assert cov_scale > 0
    k = x.shape[0]
    logdet = np.log(cov_scale) * k
    log_norm_const = -0.5 * (k * np.log(2 * np.pi) + logdet)
    return log_norm_const - 0.5 * np.sum((x - mu) * (x - mu)) / cov_scale


def logaddexp_many(vals):
    """
    Like np.logaddexp, but take a sequence instead of just two values.
    """
    result = vals[0]
    for i in xrange(1, len(vals)):
        result = np.logaddexp(result, vals[i])
    return result
