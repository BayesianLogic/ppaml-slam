package ppaml_slam

import scala.collection.JavaConversions._

import blog.common.Util
import blog.debug.ParticleFilter
import blog.model.Evidence
import blog.model.Queries
import blog.model.Model

object Main {
  def main(args: Array[String]) = {
    if (args.length != 2) {
      throw new RuntimeException(
        "Usage: Main path_to_data_dir num_particles")
    }
    val dataDirPath = args(0)
    val numParticles = args(1).toInt

    Util.initRandom(false)
    val modelPath = getClass.getResource("new.blog").getPath
    val model = new Model()
    val dummyEvidence = new Evidence(model)
    val dummyQueries = new Queries(model)
    blog.Main.simpleSetupFromFiles(model, dummyEvidence, dummyQueries, modelPath :: Nil)
    // Any evidence and queries from the model are ignored.
    // All the evidence and queries come from the feeder.
    val feeder = new SlamFeeder(model, dataDirPath)
    val pf = new ParticleFilter(model, numParticles, feeder)
    pf.advanceUntilFinished
  }
}
