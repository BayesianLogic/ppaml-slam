fast.c: fast.pyx fast_lasers.c fast_lasers.h
	python setup.py build_ext --inplace

clean:
	rm -f fast_lasers.o fast.c fast.so
	rm -rf build/

# To get an annotated html file, use `cython -a fast.pyx`.
