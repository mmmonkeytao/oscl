#ifndef PLANET_H
#define PLANET_H

#include <iostream>
#include <cstdint>
#include <Eigen/Dense>
#include <set>

namespace oscl{

  class Planet
  {
        
  public:

    typedef enum {STAR, CENTER} Type;

    // Constructor
    Planet();
    Planet(uint ID);
    
    // Destructor
    virtual ~Planet();
        
    // Public functions
    uint getID() const;
    void setID(uint uID);
        
    Type getType() const;
    void setType(Type sType);

    void print();
        
    uint getDegree() const;
    void setDegree(uint uDegree);
    void incrementDegree();
    void decrementDegree();
               
    // *****************
    // DomCenter Related
    // *****************
    uint getDomCenter();
    void setDomCenter(uint domCenterIndex);
    void setDomCenterNull();
    bool isDomCenterNull();

    // *******************
    // DomSatsList Related
    // *******************
    void insertDomSats(uint domSatIDToInsert);
    void deleteDomSats(uint domSatIDToDelete);
    void clearDomSatsList();
    bool isDomSatsListEmpty();
    std::set <uint>& getDomSatsList();
    const std::set<uint>& getConstDomSatsList() const;
    std::set <uint> getCopyOfDomSatsList();

    // *******************
    // Attraction Related
    // *******************
    void setAttr(double attr);
    double getAttr();

    void setAttrSum(double attr);
    void addAttrSum(double attr);
    void minusAttrSum(double attr);
    double getAttrSum();

    int getClustSize();
    
  private:
    //********************
    //    Private Data
    //********************
    uint _id;       // ID (useful to track data point index)
    Type _type; // Type: star or center
    uint _degree;   
    double _attraction; // attraction to cluster center
    double _attrSum; // attraction sum of stars, a center's variable
    int _domCenter;  // Id of dominant center. (-1) if null.    

    std::set<uint> _domSatsList;   // List of dominating stars

    Eigen::VectorXd centroid;
        
  };

  // Comparison operators for Vertex and Cluster Class used for sorting.
  // Sign is < here. Priority queue by default sorts be decreasing order by <.
  inline
  bool operator < (Planet a, Planet b)
  {
    return a.getDegree() < b.getDegree();
  }

  inline
  void operator << (Planet &center, Planet &star)
  {
    center.incrementDegree();
    center.insertDomSats(star.getID());

    star.setDomCenter(center.getID());
  }

  inline
  void operator >> (Planet &center, Planet &star)
  {
    center.decrementDegree();
    center.deleteDomSats(star.getID());

    star.setDomCenterNull();
  }
}

#endif // Planet.h
