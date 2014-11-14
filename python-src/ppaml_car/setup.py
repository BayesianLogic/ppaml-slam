from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext


ext_modules = [Extension("fast", ["fast.pyx"])]


setup(
    name='Fast',
    cmdclass={'build_ext': build_ext},
    ext_modules=ext_modules,
)
