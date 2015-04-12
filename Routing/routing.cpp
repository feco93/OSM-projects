/*
 * ---Route-search--- 
 * developer: Sipos Ferenc 
 * e-mail: siposferenc93@gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#include <map>
#include <list>
#include <string>
#include <iostream>
#include <cstddef>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/index/map/vector.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/geom/haversine.hpp>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp> 
#include <boost/property_map/property_map.hpp>

typedef double Weight;
typedef boost::property<boost::edge_weight_t, Weight> WeightProperty;
typedef boost::property<boost::vertex_name_t, osmium::unsigned_object_id_type> NameProperty;

typedef boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS, NameProperty, WeightProperty> Graph;
 
typedef boost::graph_traits <Graph>::vertex_descriptor Vertex;
typedef boost::property_map <Graph, boost::vertex_index_t>::type IndexMap;
typedef boost::property_map <Graph, boost::vertex_name_t>::type NameMap; 
typedef boost::iterator_property_map <Vertex*, IndexMap, Vertex, Vertex&> PredecessorMap;
typedef boost::iterator_property_map <Weight*, IndexMap, Weight, Weight&> DistanceMap;
typedef std::map<osmium::unsigned_object_id_type, Vertex> id_map_type;
typedef std::vector<Graph::edge_descriptor> PathType;

osmium::index::map::VectorBasedSparseMap <osmium::unsigned_object_id_type,osmium::Location, std::vector > locations;
      
class NearestNode : public osmium::handler::Handler
{
  const osmium::Location &loc;

  double min = 100000.0;
  osmium::unsigned_object_id_type node_id;

public:
  
  NearestNode(const osmium::Location &loc):
    loc(loc)
  {}

  void way(const osmium::Way &way)
  {
    const osmium::WayNodeList& wnl = way.nodes();
    
      for (const osmium::NodeRef& nr : wnl) 
      {
	osmium::Location temp = locations.get(nr.positive_ref());
	double dx = loc.lon() - temp.lon();
	double dy = loc.lat() - temp.lat();
	double dist = dx*dx + dy*dy;

	if (dist < min)
	{
	  min = dist;
	  node_id = nr.positive_ref();
	}
      }
    
  }

  osmium::unsigned_object_id_type get() const
  {
    return node_id;
  }
};

class Route :public osmium::handler::Handler
{
  osmium::memory::Buffer& buffer;
  Graph g;
  id_map_type id_map;
  PathType path;
  
public:
  
  Route(osmium::memory::Buffer &buffer):
    buffer(buffer)
  {}
  
  void way(const osmium::Way& way)
  {
    const osmium::WayNodeList& wnl = way.nodes();
    osmium::NodeRef prevref;
    Vertex u, v;
    
    for (const osmium::NodeRef& actref : wnl) 
    {
	id_map_type::iterator pos;
	bool inserted;
	boost::tie(pos, inserted) = id_map.emplace(actref.positive_ref(), Vertex());
        
	if (inserted) {
	  v = boost::add_vertex(actref.positive_ref(), g);
	  pos->second = v;
	}
	
	else {
	  v = id_map[actref.positive_ref()];
	}
	  
	if(prevref.positive_ref()) {
	  osmium::geom::Coordinates cord(locations.get(actref.positive_ref()));
	  osmium::geom::Coordinates prev_cord(locations.get(prevref.positive_ref()));
	  Weight w = osmium::geom::haversine::distance(cord,prev_cord);
	  boost::add_edge(u, v, w, g);
	}
	  
	prevref = actref;
	u = v;
    }  
    
  }
  
  void set_path(const osmium::Location& locs, const osmium::Location& locf) {
    
      NearestNode nodes(locs);
      osmium::apply(buffer,nodes);
      Vertex start = id_map.at(nodes.get());
      NearestNode nodef(locf);
      osmium::apply(buffer,nodef);
      Vertex finish = id_map.at(nodef.get());
      
      std::vector<Vertex> predecessors(boost::num_vertices(g));
      std::vector<Weight> distances(boost::num_vertices(g));
    
      IndexMap indexMap = boost::get(boost::vertex_index, g);
      PredecessorMap predecessorMap(&predecessors[0], indexMap);
      DistanceMap distanceMap(&distances[0], indexMap);
    
      boost::dijkstra_shortest_paths(g, start, boost::distance_map(distanceMap).predecessor_map(predecessorMap));
    
      for(Vertex u = predecessorMap[finish]; u != finish; finish = u, u = predecessorMap[finish])
      {
	std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, finish, g);
	Graph::edge_descriptor edge = edgePair.first;
	path.push_back( edge );
      }
      
    }
    
    PathType get_path() {
     return path;
    }
    
    Graph get_graph() {
      return g;
    }
  
};

int
main (int argc, char *argv[])
{
  if (argc == 2)
    {
      /*double lon,lat;
      std::cin >> lon >> lat;*/
      osmium::Location locs(21.60988,47.54164);
      //std::cin >> lon >> lat;
      osmium::Location locf(21.63773,47.52664);
      
      osmium::io::File infile (argv[1]);
      osmium::io::Reader reader (infile, osmium::osm_entity_bits::all);
      osmium::memory::Buffer buffer = reader.read();
     
      Route route(buffer);
      osmium::handler::NodeLocationsForWays < osmium::index::map::VectorBasedSparseMap <osmium::unsigned_object_id_type, osmium::Location, std::vector > >
      node_locations (locations);
      osmium::apply (buffer, node_locations);
      osmium::apply (buffer, route);
      route.set_path(locs,locf);
      
      PathType path(route.get_path());
      Graph g(route.get_graph());
      NameMap nameMap = boost::get(boost::vertex_name, g);
      PathType::reverse_iterator pathIterator = path.rbegin();
      
      if (pathIterator != path.rend()) {
	   osmium::Location loc = locations.get(nameMap[boost::source(*pathIterator, g)]);
	   std::cout <<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	   std::cout <<"<Nodes>\n";
	   std::cout <<"\t<Node lon=\"" <<loc.lon() <<"\"\tlat=\"" <<loc.lat() <<"\"/>\n";
	for(; pathIterator != path.rend(); ++pathIterator)
	{
	   loc = locations.get(nameMap[boost::target(*pathIterator, g)]);
	   std::cout <<"\t<Node lon=\"" <<loc.lon() <<"\"\tlat=\"" <<loc.lat() <<"\"/>\n";
	}
	  std::cout <<"</Nodes>\n";
      }
      else {
	std::cout <<"A keresett út nem található!\n";
      }
      
      reader.close ();      
      google::protobuf::ShutdownProtobufLibrary ();

    }
    
  else
    {
      std::cout << "Usage: " << argv[0] << "city.osm" << std::endl;
      std::exit (1);
    }
}

