/*
 *  Sipos Ferenc, siposferenc93@gmail.com
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
#include <osmium/index/map/sparse_table.hpp>
#include <osmium/index/map/vector.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/geom/haversine.hpp>

class BusHandler:public osmium::handler::Handler
{
public:
  osmium::index::map::VectorBasedSparseMap <osmium::unsigned_object_id_type,osmium::Location, std::vector > locations;
  
  void relation (osmium::Relation & rel)
  {
    const char * bus = rel.tags ()["route"];

    if (bus && (!strcmp (bus, "bus") /*|| !strcmp (bus, "trolleybus") || !strcmp(bus,"tram")*/) )
      {
	std::string bus_no;
	
	if(rel.get_value_by_key("ref"))
	{
	   bus_no=rel.get_value_by_key("ref");
	}
	else
	{
	   bus_no=rel.get_value_by_key("name");
	}
	
	std::cout <<bus_no <<" BUS"<<'\n';
	
	std::vector<std::vector<osmium::Location>> asd;
	
	for(const osmium::Tag& info : rel.tags() ) 
	{  
	  std::cout <<info.key() <<": " <<info.value() <<'\t';
	}
	std::cout <<'\n';
	
	int i=0;
	for (const osmium::RelationMember& rm: rel.members())
	  {	    
	    if(rm.type() == osmium::item_type::node) 
	    {  
		osmium::Location loc = locations.get(rm.positive_ref());
		std::cout <<i <<"\tLon: " <<loc.lon() <<"\tLat: " <<loc.lat() <<'\n';
	    }
	    ++i;
	    
	  }	  
      }

  }
};

int main (int argc, char *argv[])
{
  if (argc == 2)
    {
      osmium::io::File infile (argv[1]);
      osmium::io::Reader reader (infile, osmium::osm_entity_bits::all);

      BusHandler bus_handler;
      osmium::handler::NodeLocationsForWays < osmium::index::map::VectorBasedSparseMap <osmium::unsigned_object_id_type, osmium::Location, std::vector > >
      node_locations (bus_handler.locations);
      osmium::apply (reader, node_locations, bus_handler);
      reader.close ();
    
      google::protobuf::ShutdownProtobufLibrary ();

    }
    
  else
    {
      std::cout << "Usage: " << argv[0] << "city.osm" << std::endl;
      std::exit (1);
    }
}

