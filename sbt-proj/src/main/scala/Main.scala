import blog.debug.ParticleFilter
import ppaml_car.SlamFeeder

object Main {
  def main(args: Array[String]) = {
    if (args.length != 1) {
      throw new RuntimeException(
        "Must provide exactly one argument: the path to the dataset")
    }
    val feeder = new SlamFeeder(args(0))
    val pf = ParticleFilter.make(getClass.getResource("new.blog").getPath, 10, feeder)
    pf.advanceUntilFinished
  }
}
