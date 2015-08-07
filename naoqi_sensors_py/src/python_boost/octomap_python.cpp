/*
 * Copyright (C) 2014 Aldebaran Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Boost Python module that can convert a string to an Octomap
*/

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <octomap_msgs/conversions.h>

template<class T>
boost::python::list std_vector_to_py_list(const std::vector<T>& vec)
{
  typename std::vector<T>::const_iterator iter;
  boost::python::list res;
  for (iter = vec.begin(); iter != vec.end(); ++iter) {
    res.append(*iter);
  }
  return res;
}

/** Get a string in and return a tuple for Boost */
boost::python::tuple octomap_str_to_tuple(const std::string &str_msg)
{
  std::stringstream ss;
  ss << str_msg;
  ss.seekg(0);
  octomap::OcTree* octree = static_cast<octomap::OcTree*>(octomap::OcTree::read(ss));
  if (!octree) {
    std::cout << "Cast failed.";
    return boost::python::make_tuple();
  }
  octomap_msgs::Octomap msg;
  bool res = octomap_msgs::binaryMapToMsg(*octree, msg);
  delete(octree);
  return boost::python::make_tuple(msg.binary, msg.id, msg.resolution, std_vector_to_py_list(msg.data));
}

BOOST_PYTHON_MODULE(octomap_python)
{
  boost::python::def("octomap_str_to_tuple", octomap_str_to_tuple);
}
