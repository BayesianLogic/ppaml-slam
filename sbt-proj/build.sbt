// vim: set tw=0:

lazy val root = (project in file(".")).
  settings(
    name := "ppaml_slam",
    version := "0.2",
    scalaVersion := "2.10.4",
    libraryDependencies += "com.novocode" % "junit-interface" % "0.11" % "test",
    libraryDependencies += "com.github.tototoshi" %% "scala-csv" % "1.1.1",
    // These are BLOG's dependencies:
    libraryDependencies += "gov.nist.math" % "jama" % "1.0.3",
    libraryDependencies += "com.google.code.gson" % "gson" % "2.2.4"
    // Note: java-cup and blog itself are unmanaged dependencies, i.e. just jars in lib/.
  )
