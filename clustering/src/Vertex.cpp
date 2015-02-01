#include "Vertex.h"

using namespace std;
using namespace oscl;


/////////////////////////// CONSTRUCTOR //////////////////////////////////////////
Vertex:: Vertex()
{
  // Setting the defaults.
  _id = 10;
  _type = SATELLITE;
  _degree = 0;
  _inQ = false;
  _marked = false;
  _domCenter = -1;
}


/////////////////////////// DESTRUCTOR /////////////////////////////////////////
Vertex:: ~Vertex() 
{}

/////////////////////////// PUBLIC FUNCTIONS ////////////////////////////////////

// Accessors and mutatorsb

// ID
unsigned Vertex::getID() const
{
  return _id;
}

void Vertex::setID(unsigned uID) 
{
  _id = uID;
}

// Type
Vertex::Type Vertex::getType() const
{
  return _type;
}

void Vertex::setType(Type sType)
{
  _type = sType;
}

// Degree
unsigned Vertex::getDegree() const
{
  return _degree;
}

void Vertex::setDegree(unsigned uDegree)
{
  _degree = uDegree;
}

void Vertex::incrementDegree()
{
  _degree++;
}

void Vertex::decrementDegree()
{
  _degree--;
}

// *********************
// Adjacent Vertices List
// *********************

// Output reference to the list.
const list <unsigned>& Vertex::getAdjVerticesList() const
//list <unsigned>& Vertex::getAdjVerticesList() 
{
  return _adjVerticesList;
}

// Obtain the whole list.
list <unsigned> Vertex::getCopyOfAdjVerticesList() const
{
  return _adjVerticesList;
}

// Set the whole list
void Vertex::setAdjVerticesList(list <unsigned> adjVerticesList)
{
  _adjVerticesList = adjVerticesList;
}

// Insert a vertex into the adjacency list.
void Vertex::insertAdjVertex(unsigned adjVertexIDToInsert)
{
  _adjVerticesList.push_back(adjVertexIDToInsert);
}

// Delete a vertex from the adjacency list.
void Vertex::deleteAdjVertex(unsigned adjVertexIDToDelete)
{
  _adjVerticesList.remove(adjVertexIDToDelete);
}

// *********************
// Adjacent Centers List
// *********************

// Output reference
list <unsigned>& Vertex::getAdjCentersList()
{
  return _adjCentersList;
}

// Output whole list
list <unsigned> Vertex::getCopyOfAdjCentersList()
{
  return _adjCentersList;
}

void Vertex::setAdjCenters(list <unsigned> adjCentersList)
{
  _adjCentersList = adjCentersList;
}

// Insert a vertex into the adjacency list.
void Vertex::insertAdjCenter(unsigned adjCenterIDToInsert)
{
  _adjCentersList.push_back(adjCenterIDToInsert);
}

// Delete a vertex from the adjacency list.
void Vertex::deleteAdjCenter(unsigned adjCenterIDToDelete)
{
  _adjCentersList.remove(adjCenterIDToDelete);
}

// Is adjacent centers list empty?
bool Vertex::isAdjCentersListEmpty()
{
  return _adjCentersList.empty();
}

// Boolean status to indicate presence in the queue.
bool Vertex::getInQStatus() 
{
  return _inQ;
}

void Vertex::setInQStatus(bool bInQStatus)
{
  _inQ = bInQStatus;
}

// Marked
bool Vertex::getMarkedStatus()
{
  return _marked;
}

void Vertex::setMarkedStatus(bool bMarkedStatus)
{
  _marked = bMarkedStatus;
}

void Vertex::clearCentersList()
{
  _adjCentersList.clear();
}

//********** RELATED TO OPTIMIZED VERSIONS ********

// Related to domcenter.

unsigned Vertex::getDomCenter()
{
  return _domCenter;
}

void Vertex::setDomCenter(unsigned domCenterIndex)
{
  _domCenter = domCenterIndex;
}

void Vertex::setDomCenterNull()
{
  _domCenter = -1;
}

bool Vertex::isDomCenterNull()
{
  return (_domCenter == -1) ? true : false;
}

// Related to domSatsList

// Insert a satellite ID into the dominant satellite list.
void Vertex::insertDomCenter(unsigned domSatIDToInsert)
{
  _domSatsList.push_back(domSatIDToInsert);
}

// Delete a satellite ID into the dominant satellite list.
void Vertex::deleteDomCenter(unsigned domSatIDToDelete)
{
  _domSatsList.remove(domSatIDToDelete);
}

// Clear the dominant centers list.
void Vertex::clearDomCentersList()
{
  _domSatsList.clear();
}

// Is dom centers list empty?
bool Vertex::isDomSatsListEmpty()
{
  return _domSatsList.empty();
}

// Output reference to the dom centers list.
list <unsigned>& Vertex::getDomSatsList()
{
  return _domSatsList;
}

// Obtain the whole dom centers list.
list <unsigned> Vertex::getCopyOfDomSatsList()
{
  return _domSatsList;
}


// Printing a vector
void Vertex::print()
{

  std::cout<< "VertexID: " << _id << " " << std::endl;
  std::cout<< "VertexType: " << _type << " ";
  std::cout<< "Degree: " << _degree << std::endl;
  std::cout<< "DomCenter: " << _domCenter << std::endl;
    
  std::cout << "adjVerticesList: ";
  copy(_adjVerticesList.begin(), _adjVerticesList.end(), ostream_iterator <unsigned> (cout, " "));
  std::cout << std::endl;
    
  std::cout << "adjCentersList: ";
  copy(_adjCentersList.begin(), _adjCentersList.end(), ostream_iterator <unsigned> (cout, " "));
  std::cout << std::endl;
     
  std::cout << "domSatsList: ";
  copy(_domSatsList.begin(), _domSatsList.end(), ostream_iterator <unsigned> (cout, " "));
  std::cout << std::endl;
    
  std::cout<< " Priority Queue status: " << _inQ << " Marked: "<< _marked <<std::endl;     
}

