package jiglib.math;

class JMath3D
{
    public static inline var NUM_TINY : Float = 0.000001;
    public static inline var NUM_HUGE : Float = 1000000;
    
    public static function fromNormalAndPoint(normal : Vector3D, point : Vector3D) : Vector3D
    {
        var v : Vector3D = new Vector3D(normal.x, normal.y, normal.z);
        v.w = -(v.x * point.x + v.y * point.y + v.z * point.z);
        
        return v;
    }
    
    public static function getIntersectionLine(v : Vector3D, v0 : Vector3D, v1 : Vector3D) : Vector3D
    {
        var d0 : Float = v.x * v0.x + v.y * v0.y + v.z * v0.z - v.w;
        var d1 : Float = v.x * v1.x + v.y * v1.y + v.z * v1.z - v.w;
        var m : Float = d1 / (d1 - d0);
        return new Vector3D(
        v1.x + (v0.x - v1.x) * m, 
        v1.y + (v0.y - v1.y) * m, 
        v1.z + (v0.z - v1.z) * m);
    }
    
    public static function wrap(val : Float, min : Float, max : Float) : Float
    {
        var delta : Float = max - min;
        if (val > delta) 
        {
            val = val / delta;
            //val = val - Std.parseFloat(Math.floor(val));
            val = val - (Math.floor(val));
            val = val * delta;
        }
        return val;
    }
    
    public static function getLimiteNumber(num : Float, min : Float, max : Float) : Float
    {
        var n : Float = num;
        if (n < min) 
        {
            n = min;
        }
        else if (n > max) 
        {
            n = max;
        }
        return n;
    }
}
