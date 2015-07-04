package jiglib.data;


// structure used to set up the mesh
class TriangleVertexIndices
{
    
    public var i0 : Int;
    public var i1 : Int;
    public var i2 : Int;
    
    public function new(_i0 : Int, _i1 : Int, _i2 : Int)
    {
        this.i0 = _i0;
        this.i1 = _i1;
        this.i2 = _i2;
    }
}
