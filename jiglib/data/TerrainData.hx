package jiglib.data;


import jiglib.math.Vector3D;

class TerrainData
{
    public var height : Float;
    public var normal : Vector3D;
    
    public function new(height : Float = 0, normal : Vector3D = null)
    {
        this.height = (Math.isNaN(height)) ? 0 : height;
        this.normal = (normal != null) ? normal : new Vector3D();
    }
}
