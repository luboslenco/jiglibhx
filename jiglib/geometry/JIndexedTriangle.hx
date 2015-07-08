package jiglib.geometry;

import jiglib.data.PlaneData;
import jiglib.math.JNumber3D;
import jiglib.math.Vector3D;

/// Support for an indexed triangle - assumes ownership by something that
/// has an array of vertices and an array of tIndexedTriangle
class JIndexedTriangle
{
    public var vertexIndices(get, never) : Array<Int>;
    public var plane(get, never) : PlaneData;
    public var boundingBox(get, never) : JAABox;

    public var counter : Int;
    
    private var _vertexIndices : Array<Int>;
    private var _plane : PlaneData;
    private var _boundingBox : JAABox;
    
    public function new()
    {
        counter = 0;
        _vertexIndices = [-1, -1, -1];
        _plane = new PlaneData();
        _boundingBox = new JAABox();
    }
    
    /// Set the indices into the relevant vertex array for this triangle. Also sets the plane and bounding box
    public function setVertexIndices(i0 : Int, i1 : Int, i2 : Int, vertexArray : Array<Vector3D>) : Void{
        _vertexIndices[0] = i0;
        _vertexIndices[1] = i1;
        _vertexIndices[2] = i2;
        
        _plane.setWithPoint(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
        
        _boundingBox.clear();
        _boundingBox.addPoint(vertexArray[i0]);
        _boundingBox.addPoint(vertexArray[i1]);
        _boundingBox.addPoint(vertexArray[i2]);
    }
    
    public function updateVertexIndices(vertexArray : Array<Vector3D>) : Void{
        var i0 : Int;
        var i1 : Int;
        var i2 : Int;
        i0 = _vertexIndices[0];
        i1 = _vertexIndices[1];
        i2 = _vertexIndices[2];
        
        _plane.setWithPoint(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
        
        _boundingBox.clear();
        _boundingBox.addPoint(vertexArray[i0]);
        _boundingBox.addPoint(vertexArray[i1]);
        _boundingBox.addPoint(vertexArray[i2]);
    }
    
    // Get the indices into the relevant vertex array for this triangle.
    private function get_vertexIndices() : Array<Int>{
        return _vertexIndices;
    }
    
    // Get the vertex index association with iCorner (which should be 0, 1 or 2)
    public function getVertexIndex(iCorner : Int) : Int{
        return _vertexIndices[iCorner];
    }
    
    // Get the triangle plane
    private function get_plane() : PlaneData{
        return _plane;
    }
    
    private function get_boundingBox() : JAABox{
        return _boundingBox;
    }
}
