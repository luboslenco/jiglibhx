package jiglib.data;

import jiglib.geometry.JAABox;
import jiglib.math.JNumber3D;
import jiglib.math.Vector3D;

class OctreeCell
{
    public var points(get, never) : Array<Vector3D>;
    public var egdes(get, never) : Array<EdgeData>;

    
    public static var NUM_CHILDREN : Int = 8;
    
    // indices of the children (if not leaf). Will be -1 if there is no child
    public var childCellIndices : Array<Int>;
    // indices of the triangles (if leaf)
    public var triangleIndices : Array<Int>;
    // Bounding box for the space we own
    public var AABox : JAABox;
    
    private var _points : Array<Vector3D>;
    private var _egdes : Array<EdgeData>;
    
    public function new(aabox : JAABox)
    {
        childCellIndices = [for (i in 0...NUM_CHILDREN) -1];
        triangleIndices = new Array<Int>();
        
        clear();
        
        if (aabox != null) {
            AABox = aabox.clone();
        }
        else {
            AABox = new JAABox();
        }
        _points = AABox.getAllPoints();
        _egdes = AABox.edges;
    }
    
    // Indicates if we contain triangles (if not then we should/might have children)
    public function isLeaf() : Bool{
        return childCellIndices[0] == -1;
    }
    
    public function clear() : Void{
        for (i in 0...NUM_CHILDREN){
            childCellIndices[i] = -1;
        }
        triangleIndices.splice(0, triangleIndices.length);
    }
    
    private function get_points() : Array<Vector3D>{
        return _points;
    }
    private function get_egdes() : Array<EdgeData>{
        return _egdes;
    }
}

