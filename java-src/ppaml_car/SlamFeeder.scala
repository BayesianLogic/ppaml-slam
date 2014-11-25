package ppaml_car

import blog.debug.FilterFeeder

class SlamFeeder(dirPath: String) extends FilterFeeder {

  def hasNext: Boolean = ???

  def next: (Int, blog.model.Evidence, blog.model.Queries) = ???

}
