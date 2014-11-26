package ppaml_slam

import scala.collection.JavaConversions._

import blog.common.Util
import blog.debug.ParticleFilter
import blog.model.Evidence
import blog.model.Queries
import blog.model.Model

object Main {
  def main(args: Array[String]) = {
    if (args.length != 1) {
      throw new RuntimeException(
        "Must provide exactly one argument: the path to the dataset")
    }
    Util.initRandom(false)

    val modelPath = getClass.getResource("new.blog").getPath
    val model = new Model()
    val dummyEvidence = new Evidence(model)
    val dummyQueries = new Queries(model)
    blog.Main.simpleSetupFromFiles(model, dummyEvidence, dummyQueries, modelPath :: Nil)
    // Any evidence and queries from the model are ignored.
    // All the evidence and queries come from the feeder.
    val feeder = new SlamFeeder(model, args(0))
    val pf = new ParticleFilter(model, 10, feeder)
    pf.advanceUntilFinished
  }
}
