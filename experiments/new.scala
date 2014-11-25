import blog.debug.ParticleFilter
import ppaml_car.SlamFeeder

val feeder = new SlamFeeder(args(0))
val pf = ParticleFilter.make("new.blog", 10, feeder)
pf.advanceUntilFinished
