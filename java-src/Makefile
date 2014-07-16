# This expects the CLASSPATH env var to be set properly.
# setup_env does this.

all: laser_standalone laser_glue dynamics_standalone dynamics_glue

laser_standalone: ppaml_car/LaserLogic.java ppaml_car/TestLaserLogic.java
	javac \
		ppaml_car/LaserLogic.java \
		ppaml_car/TestLaserLogic.java

laser_glue: ppaml_car/LaserLogic.java ppaml_car/TestLaserLogic.java ppaml_car/LaserInterp.java
	javac \
		ppaml_car/LaserLogic.java \
		ppaml_car/TestLaserLogic.java \
		ppaml_car/LaserInterp.java

dynamics_standalone: ppaml_car/DynamicsLogic.java ppaml_car/TestDynamicsLogic.java
	javac \
		ppaml_car/DynamicsLogic.java \
		ppaml_car/TestDynamicsLogic.java

dynamics_glue: ppaml_car/DynamicsLogic.java ppaml_car/TestDynamicsLogic.java ppaml_car/DynamicsInterp.java
	javac \
		ppaml_car/DynamicsLogic.java \
		ppaml_car/TestDynamicsLogic.java \
		ppaml_car/DynamicsInterp.java

test:
	java \
		org.junit.runner.JUnitCore \
		ppaml_car.TestLaserLogic \
		ppaml_car.TestDynamicsLogic
