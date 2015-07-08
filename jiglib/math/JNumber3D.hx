package jiglib.math;

class JNumber3D
{
    public static function toArray(v : Vector3D) : Array<Float>
    {
        var arr : Array<Float> = [v.x, v.y, v.z];
        return arr;
    }
    
    public static function copyFromArray(v : Vector3D, arr : Array<Float>) : Void
    {
        if (arr.length >= 3) 
        {
            v.x = arr[0];
            v.y = arr[1];
            v.z = arr[2];
        }
    }
    
    public static function getScaleVector(v : Vector3D, s : Float) : Vector3D
    {
        return new Vector3D(v.x * s, v.y * s, v.z * s, v.w);
    }
    
    public static function getDivideVector(v : Vector3D, w : Float) : Vector3D
    {
        if (w != 0) 
        {
            return new Vector3D(v.x / w, v.y / w, v.z / w);
        }
        else 
        {
            return new Vector3D(0, 0, 0);
        }
    }
    
    public static function getNormal(v0 : Vector3D, v1 : Vector3D, v2 : Vector3D) : Vector3D
    {
        var E : Vector3D = v1.clone();
        var F : Vector3D = v2.clone();
        var N : Vector3D = E.subtract(v0).crossProduct(F.subtract(v1));
        N.normalize();
        
        return N;
    }
}
