from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

# Major thanks to http://stackoverflow.com/a/3071942/744071.

ext_modules = [Extension(
    name="fast",
    sources=["fast.pyx", "fast_lasers.c"],
    extra_compile_args=["-std=c99"],
    undef_macros=['NDEBUG'],
)]


setup(
    name='Fast',
    cmdclass={'build_ext': build_ext},
    ext_modules=ext_modules,
)
