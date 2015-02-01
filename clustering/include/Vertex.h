#ifndef VERTEX_H
#define VERTEX_H

#include <vector>
#include <list>
#include <iostream>
#include <iterator>
#include <cstdint>

namespace oscl{

  class Vertex
  {
        
  public:

    typedef enum {SATELLITE, CENTER} Type;

    // Constructor
    Vertex ();
        
    // Destructor
    virtual ~Vertex();
        
    // Public functions
    uint getID() const;
    void setID(uint uID);
        
    Type getType() const;
    void setType(Type sType);
        
    uint getDegree() const;
    void setDegree(uint uDegree);
    void incrementDegree();
    void decrementDegree();
        
    // *********************
    // Adjacent Vertices List
    // *********************
    const std::list <uint>& getAdjVerticesList() const;
    //std::list <uint>& getAdjVerticesList();
    std::list <uint> getCopyOfAdjVerticesList() const;        
    void setAdjVerticesList(std::list <uint> adjVerticesList);
    void insertAdjVertex(uint adjVertexIDToInsert);
    void deleteAdjVertex(uint adjVertexIDToDelete);
        
    // *********************
    // Adjacent Centers List
    // *********************
    std::list <uint>& getAdjCentersList();
    std::list <uint> getCopyOfAdjCentersList();
    void setAdjCenters(std::list <uint> adjCentersList);        
    void insertAdjCenter(uint adjCenterIDToInsert);
    void deleteAdjCenter(uint adjCenterIDToDelete);
    bool isAdjCentersListEmpty();
    void clearCentersList(); 

    // *********************
    // Priority Queue status related.
    // *********************
    bool getInQStatus();
    void setInQStatus(bool bInQStatus);
        
    bool getMarkedStatus();
    void setMarkedStatus(bool bMarkedStatus);
        
    void print();
       
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
    void insertDomCenter(uint domSatIDToInsert);
    void deleteDomCenter(uint domSatIDToDelete);
    void clearDomCentersList();
    bool isDomSatsListEmpty();
    std::list <uint>& getDomSatsList();
    std::list <uint> getCopyOfDomSatsList();
        
  private:
    //********************
    //    Private Data
    //********************
    uint _id;       // ID (useful to track data point index)
    Type _type; // Type: satellite or center
    uint _degree;   // Degree
        
    std::list<uint> _adjVerticesList;     // adjacent vertices list.        
    std::list<uint> _adjCentersList;      // adjacent centers list.        
        
    bool _inQ;          // Status in Q.
    bool _marked;       // Marked.
        
    // Extra fields for optimized versions.
    std::list<uint> _domSatsList;   // List of dominating centers.
    int _domCenter;        // Id of dominant center. (-1) if null.
        
  };

  // Comparison operators for Vertex and Cluster Class used for sorting.
  // Sign is < here. Priority queue by default sorts be decreasing order by <.
  inline
  bool operator < (Vertex a, Vertex b)
  {
    return a.getDegree() < b.getDegree();
  }

}

#endif //VERTEX_H





