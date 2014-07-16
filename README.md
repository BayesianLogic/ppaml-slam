# PPAML CP#1: SLAM

Directory structure:

- `blog`: submodule containing BLOG
- `data`: symlink to data dir
- `java-src`: dynamics and observation functions
- `python-src`: code for generating the model from data
- `experiments`: stuff that's in flux all the time


## Setting up your environment

Here is an example initial setup on a Ubuntu machine. You only have to do this
once. You can skip some steps; for example if you don't want to use a
virtualenv, you can install the Python packages system-wide, and if you already
have sbt you don't need to reinstall it.

```
cd ppaml-slam

# Create Python virtualenv and install requirements.
mkvirtualenv ppaml-slam
setvirtualenvproject
pip install -r requirements.txt

# Check out and compile BLOG submodule.
git submodule update --init
cd blog
sudo apt-get install sbt
sbt compile stage
cd ..

# Make sure data/ is a symlink to the data dir.
# It should contain directories `1_straight` etc.
ln -s ....path-to-data.... data
```

You have to do the following in every terminal. This will set up the
appropriate `PYTHONPATH` and `CLASSPATH` so that all the components are found.

```
source setup_env
```


## Compiling the Java components

```
cd java-src

# Compile:
make

# Run unit tests:
make test
```


## Running on a dataset

```
cd experiments

# Generate car.blog in the current directory:
python -m ppaml_car.blog_gen 2_bend noisy

# Run BLOG particle filter and write out.json:
../blog/blog -e blog.engine.ParticleFilter -n 100 -r -o out.json car.blog

# Evaluate results:
python -m ppaml_car.evaluate 2_bend out.json --plot
```
