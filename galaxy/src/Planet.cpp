#include "Planet.h"

using namespace std;
using namespace oscl;

/////////////////////////// CONSTRUCTOR //////////////////////////////////////////
Planet:: Planet()
{
  _domCenter = -1;
  _domSatsList = std::set<uint>();
  _degree = 0;
  _attrSum = 0.0;
  setDomCenterNull();
  _adjPlanetsList = std::set<uint>();
  _adjCentersList = std::set<uint>();
}

Planet:: Planet(uint ID)
{
  _domCenter = -1;
  _domSatsList = std::set<uint>();
  _degree = 0;
  _attrSum = 0.0;
  setDomCenterNull();
  setID(ID);
  setType(Planet::STAR);
  _adjPlanetsList = std::set<uint>();
  _adjCentersList = std::set<uint>();
}


/////////////////////////// DESTRUCTOR /////////////////////////////////////////
Planet:: ~Planet() 
{}

/////////////////////////// PUBLIC FUNCTIONS ////////////////////////////////////

// Accessors and mutatorsb

// ID
uint Planet::getID() const
{
  return _id;
}

void Planet::setID(uint uID) 
{
  _id = uID;
}

// Type
Planet::Type Planet::getType() const
{
  return _type;
}

void Planet::setType(Type sType)
{
  _type = sType;
}

// Degree
uint Planet::getDegree() const
{
  return _degree;
}

void Planet::setDegree(uint uDegree)
{
  _degree = uDegree;
}

void Planet::incrementDegree()
{
  ++_degree;
}

void Planet::decrementDegree()
{
  --_degree;
}


//********** RELATED TO OPTIMIZED VERSIONS ********

// Related to domcenter.

uint Planet::getDomCenter()
{
  return _domCenter;
}

void Planet::setDomCenter(uint domCenterIndex)
{
  _domCenter = domCenterIndex;
}

void Planet::setDomCenterNull()
{
  _domCenter = -1;
}

bool Planet::isDomCenterNull()
{
  return (_domCenter == -1) ? true : false;
}

// Related to domSatsList

// Insert a satellite ID into the dominant satellite list.
void Planet::insertDomSats(uint domSatIDToInsert)
{
  _domSatsList.insert(domSatIDToInsert);
}

// Delete a satellite ID into the dominant satellite list.
void Planet::deleteDomSats(uint domSatIDToDelete)
{
  _domSatsList.erase(domSatIDToDelete);
}

// Clear the dominant centers list.
void Planet::clearDomSatsList()
{
  _domSatsList.clear();
}

// Is dom centers list empty?
bool Planet::isDomSatsListEmpty()
{
  return _domSatsList.empty();
}

// Output reference to the dom centers list.
set<uint>& Planet::getDomSatsList()
{
  return _domSatsList;
}

// Obtain the whole dom centers list.
set<uint> Planet::getCopyOfDomSatsList()
{
  return _domSatsList;
}

const set<uint>& Planet::getConstDomSatsList() const
{
  return _domSatsList;
}

void Planet::setAttr(double attr) {
  _attraction = attr;
}

double Planet::getAttr() {
  return _attraction;
}

void Planet::setAttrSum(double attr) {
  _attrSum = attr;
}

void Planet::addAttrSum(double attr) {
  _attrSum += attr;
}

void Planet::minusAttrSum(double attr){
  _attrSum -= attr;
}

double Planet::getAttrSum() {
  return _attrSum;
}

// *********************
// Adjacent Vertices List
// *********************

// Output reference to the list.
const set<uint>& Planet::getAdjPlanetsList() const
{
  return _adjPlanetsList;
}

// Obtain the whole list.
set<uint> Planet::getCopyOfAdjPlanetsList() const
{
  return _adjPlanetsList;
}

// Insert a vertex into the adjacency list.
void Planet::insertAdjPlanet(uint adjPlanetIDToInsert)
{
  _adjPlanetsList.insert(adjPlanetIDToInsert);
}

// Delete a vertex from the adjacency list.
void Planet::deleteAdjPlanet(uint adjPlanetIDToDelete)
{
  _adjPlanetsList.erase(adjPlanetIDToDelete);
}

// *********************
// Adjacent Centers List
// *********************
// Output reference
set<uint>& Planet::getAdjCentersList()
{
  return _adjCentersList;
}

// Output whole list
set <uint> Planet::getCopyOfAdjCentersList()
{
  return _adjCentersList;
}

// Insert a vertex into the adjacency list.
void Planet::insertAdjCenter(uint adjCenterIDToInsert)
{
  _adjCentersList.insert(adjCenterIDToInsert);
}

// Delete a vertex from the adjacency list.
void Planet::deleteAdjCenter(uint adjCenterIDToDelete)
{
  _adjCentersList.erase(adjCenterIDToDelete);
}

// Is adjacent centers list empty?
bool Planet::isAdjCentersListEmpty()
{
  return _adjCentersList.empty();
}

void Planet::clearCentersList()
{
  _adjCentersList.clear();
}

// Boolean status to indicate presence in the queue.
bool Planet::getInQStatus() 
{
  return _inQ;
}

void Planet::setInQStatus(bool bInQStatus)
{
  _inQ = bInQStatus;
}

int Planet::getClustSize() {

  if(_type == Planet::CENTER)
    return _degree + 1;
  else{
    std::cerr << "This is a Stars, please ask the center!\n";
    return -1;
  }
}

// Printing a vector
void Planet::print()
{

  std::cout<< "PlanetID: " << _id << " " << std::endl;
  std::cout<< "PlanetType: " << _type << " ";
  std::cout<< "Degree: " << _degree << std::endl;
  std::cout<< "DomCenter: " << _domCenter << std::endl;
         
  std::cout << "domSatsList: ";
  copy(_domSatsList.begin(), _domSatsList.end(), ostream_iterator <uint> (cout, " "));
  std::cout << std::endl;
    
}

