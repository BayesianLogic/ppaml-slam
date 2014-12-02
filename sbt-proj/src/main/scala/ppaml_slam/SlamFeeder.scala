package ppaml_slam

import java.io.File

import blog.debug.FilterFeeder
import blog.model.Evidence
import blog.model.Queries
import blog.model.Model
import com.github.tototoshi.csv._

class SlamFeeder(model: Model, dirPath: String) extends FilterFeeder {

  // Read properties file.
  val propertiesReader = CSVReader.open(new File(
    dirPath + "/input/input_properties.csv")).iterator
  val propertiesHeader = propertiesReader.next
  val propertiesValues = propertiesReader.next
  val paramL = propertiesValues(0).toDouble
  val paramH = propertiesValues(1).toDouble
  val paramA = propertiesValues(2).toDouble
  val paramB = propertiesValues(3).toDouble
  val initTheta = propertiesValues(4).toDouble
  val initY = propertiesValues(5).toDouble
  val initX = propertiesValues(6).toDouble

  // Set up readers for the sensor files.
  // These will be read lazily (in an online fashion).
  val sensorReader = CSVReader.open(new File(
    dirPath + "/input/input_sensor.csv")).iterator
  val code2sensor = Map("1" -> 'gps, "2" -> 'control, "3" -> 'laser)
  val sensorHeader = sensorReader.next
  val controlReader = CSVReader.open(new File(
    dirPath + "/input/input_control.csv")).iterator
  val controlHeader = controlReader.next
  val laserReader = CSVReader.open(new File(
    dirPath + "/input/input_laser.csv")).iterator
  val laserHeader = laserReader.next

  // Read ground obstacles for now (this is cheating).
  val obstaclesReader = CSVReader.open(new File(
    dirPath + "/eval_data/eval_obstacles.csv")).iterator
  val obstaclesHeader = obstaclesReader.next
  val obstacleRadius = 0.37
  val obstacles = obstaclesReader.toList.map(obstaclesLineToObstacle)

  var timestep = -1

  def hasNext: Boolean = sensorReader.hasNext

  def next: (Int, Evidence, Queries) = {
    val evidence = new Evidence(model)
    val queries = new Queries(model)
    val result = if (timestep == -1) {
      // Return atemporal stuff.
      evidence.addFromString(
        s"obs carParams = [ $paramL; $paramH; $paramA; $paramB ];")
      evidence.addFromString(
        s"obs initialState = [ $initX; $initY; $initTheta ];")
      val obstaclesBlogStr = seqSeqToBlogMatrix(obstacles)
      evidence.addFromString(s"obs obstacles = $obstaclesBlogStr;")
    } else {
      // Return next timestep from the dataset.
      val sensorLine = sensorReader.next
      val time = sensorLine(0).toDouble
      val sensor = code2sensor(sensorLine(1))
      evidence.addFromString(s"obs time(@$timestep) = $time;")
      if (sensor == 'gps) {
        evidence.addFromString(s"obs haveControls(@$timestep) = false;")
        queries.addFromString(s"query time(@$timestep);")
        queries.addFromString(s"query stateWithoutNoise(@$timestep);")
      } else if (sensor == 'control) {
        val controlLine = controlReader.next
        val controlTime = controlLine(0).toDouble
        val velocity = controlLine(1).toDouble
        val steering = controlLine(2).toDouble
        assert (Math.abs(controlTime - time) < 1e-2)
        evidence.addFromString(s"obs haveControls(@$timestep) = true;")
        evidence.addFromString(s"obs velocity(@$timestep) = $velocity;")
        evidence.addFromString(s"obs steering(@$timestep) = $steering;")
      } else if (sensor == 'laser) {
        val laserLine = laserReader.next
        val laserTime = laserLine(0).toDouble
        val laserVals = laserLine.drop(1).take(361).map((s) => s.toDouble)
        evidence.addFromString(s"obs haveControls(@$timestep) = false;")
        val laserBlogStr = seqToBlogColVec(laserVals)
        assert (Math.abs(laserTime - time) < 1e-2)
        evidence.addFromString(
          s"obs laserReadings(@$timestep) = $laserBlogStr;")
      }
    }
    evidence.compile  // FIXME: clumsy interface
    queries.compile  // FIXME: clumsy interface
    timestep += 1
    println("-------------")
    println(timestep - 1)
    println(evidence)
    println(queries)
    println("-------------")
    (timestep - 1, evidence, queries)
  }

  def obstaclesLineToObstacle(line: Seq[String]): Seq[Double] = {
    Seq(line(0).toDouble, line(1).toDouble, obstacleRadius)
  }

  def seqToBlogColVec(seq: Seq[Double]): String = {
    "[ " + seq.mkString("; ") + "]"
  }

  def seqToBlogRowVec(seq: Seq[Double]): String = {
    "[ " + seq.mkString(", ") + "]"
  }

  def seqSeqToBlogMatrix(seqSeq: Seq[Seq[Double]]): String = {
    "[\n    " + seqSeq.map(seqToBlogRowVec).mkString(",\n    ") + "\n]"
  }

}
