package jiglib.geometry;

import jiglib.data.CollOutData;
import jiglib.data.PlaneData;
import jiglib.data.SpanData;
import jiglib.math.JNumber3D;
import jiglib.math.JMath3D;
import jiglib.math.Vector3D;

// Defines a 3d triangle. Each edge goes from the origin. Cross(edge0, edge1)  gives the triangle normal.
class JTriangle
{
    public var edge2(get, never) : Vector3D;
    public var normal(get, never) : Vector3D;
    public var plane(get, never) : PlaneData;

    public var origin : Vector3D;
    public var edge0 : Vector3D;
    public var edge1 : Vector3D;
    
    // Points specified so that pt1-pt0 is edge0 and p2-pt0 is edge1
    public function new(pt0 : Vector3D, pt1 : Vector3D, pt2 : Vector3D)
    {
        origin = pt0.clone();
        edge0 = pt1.subtract(pt0);
        edge1 = pt2.subtract(pt0);
    }
    
    // Edge2 goes from pt1 to pt2
    private function get_edge2() : Vector3D{
        return edge1.subtract(edge0);
    }
    
    // Gets the triangle normal.
    private function get_normal() : Vector3D{
        var N : Vector3D = edge0.crossProduct(edge1);
        N.normalize();
        
        return N;
    }
    
    // Gets the plane containing the triangle
    private function get_plane() : PlaneData{
        var pl : PlaneData = new PlaneData();
        pl.setWithNormal(origin, normal);
        
        return pl;
    }
    
    // Returns the point parameterised by t0 and t1
    public function getPoint(t0 : Float, t1 : Float) : Vector3D{
        var d0 : Vector3D;
        var d1 : Vector3D;
        d0 = edge0.clone();
        d1 = edge1.clone();
        
        d0.scaleBy(t0);
        d1.scaleBy(t1);
        
        return origin.add(d0).add(d1);
    }
    
    public function getCentre() : Vector3D{
        var result : Vector3D = edge0.add(edge1);
        result.scaleBy(0.333333);
        
        return origin.add(result);
    }
    
    // Same numbering as in the constructor
    public function getVertex(_id : Int) : Vector3D{
        switch (_id)
        {
            case 1:
                return origin.add(edge0);
            case 2:
                return origin.add(edge1);
            default:
                return origin;
        }
    }
    public function getSpan(axis : Vector3D) : SpanData{
        var d0 : Float;
        var d1 : Float;
        var d2 : Float;
        d0 = getVertex(0).dotProduct(axis);
        d1 = getVertex(1).dotProduct(axis);
        d2 = getVertex(2).dotProduct(axis);
        
        var result : SpanData = new SpanData();
        result.min = Math.min(d0, Math.min(d1, d2));
        result.max = Math.max(d0, Math.max(d1, d2));
        
        return result;
    }
    
    public function segmentTriangleIntersection(out : CollOutData, seg : JSegment) : Bool{
        
        var u : Float;
        var v : Float;
        var t : Float;
        var a : Float;
        var f : Float;
        var p : Vector3D;
        var s : Vector3D;
        var q : Vector3D;
        
        p = seg.delta.crossProduct(edge1);
        a = edge0.dotProduct(p);
        
        if (a > -JMath3D.NUM_TINY && a < JMath3D.NUM_TINY) {
            return false;
        }
        f = 1 / a;
        s = seg.origin.subtract(origin);
        u = f * s.dotProduct(p);
        
        if (u < 0 || u > 1)             return false;
        
        q = s.crossProduct(edge0);
        v = f * seg.delta.dotProduct(q);
        if (v < 0 || (u + v) > 1)             return false;
        
        t = f * edge1.dotProduct(q);
        if (t < 0 || t > 1)             return false;
        
        if (out != null)             out.frac = t;
        return true;
    }
    
    public function pointTriangleDistanceSq(out : Array<Float>, point : Vector3D) : Float{
        
        var fA00 : Float;
        var fA01 : Float;
        var fA11 : Float;
        var fB0 : Float;
        var fB1 : Float;
        var fC : Float;
        var fDet : Float;
        var fS : Float;
        var fT : Float;
        var fSqrDist : Float;
        
        var kDiff : Vector3D = origin.subtract(point);
        fA00 = edge0.lengthSquared;
        fA01 = edge0.dotProduct(edge1);
        fA11 = edge1.lengthSquared;
        fB0 = kDiff.dotProduct(edge0);
        fB1 = kDiff.dotProduct(edge1);
        fC = kDiff.lengthSquared;
        fDet = Math.abs(fA00 * fA11 - fA01 * fA01);
        fS = fA01 * fB1 - fA11 * fB0;
        fT = fA01 * fB0 - fA00 * fB1;
        
        if (fS + fT <= fDet) 
        {
            if (fS < 0) 
            {
                if (fT < 0)   // region 4  
                {
                    if (fB0 < 0) 
                    {
                        fT = 0;
                        if (-fB0 >= fA00) 
                        {
                            fS = 1;
                            fSqrDist = fA00 + 2 * fB0 + fC;
                        }
                        else 
                        {
                            fS = -fB0 / fA00;
                            fSqrDist = fB0 * fS + fC;
                        }
                    }
                    else 
                    {
                        fS = 0;
                        if (fB1 >= 0) 
                        {
                            fT = 0;
                            fSqrDist = fC;
                        }
                        else if (-fB1 >= fA11) 
                        {
                            fT = 1;
                            fSqrDist = fA11 + 2 * fB1 + fC;
                        }
                        else 
                        {
                            fT = -fB1 / fA11;
                            fSqrDist = fB1 * fT + fC;
                        }
                    }
                }
                // region 3
                else 
                {
                    fS = 0;
                    if (fB1 >= 0) 
                    {
                        fT = 0;
                        fSqrDist = fC;
                    }
                    else if (-fB1 >= fA11) 
                    {
                        fT = 1;
                        fSqrDist = fA11 + 2 * fB1 + fC;
                    }
                    else 
                    {
                        fT = -fB1 / fA11;
                        fSqrDist = fB1 * fT + fC;
                    }
                }
            }
            else if (fT < 0)   // region 5  
            {
                fT = 0;
                if (fB0 >= 0) 
                {
                    fS = 0;
                    fSqrDist = fC;
                }
                else if (-fB0 >= fA00) 
                {
                    fS = 1;
                    fSqrDist = fA00 + 2 * fB0 + fC;
                }
                else 
                {
                    fS = -fB0 / fA00;
                    fSqrDist = fB0 * fS + fC;
                }
            }
            // region 0
            else 
            {
                // minimum at interior point
                var fInvDet : Float = 1 / fDet;
                fS *= fInvDet;
                fT *= fInvDet;
                fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
            }
        }
        else 
        {
            var fTmp0 : Float;
            var fTmp1 : Float;
            var fNumer : Float;
            var fDenom : Float;
            
            if (fS < 0)   // region 2  
            {
                fTmp0 = fA01 + fB0;
                fTmp1 = fA11 + fB1;
                if (fTmp1 > fTmp0) 
                {
                    fNumer = fTmp1 - fTmp0;
                    fDenom = fA00 - 2 * fA01 + fA11;
                    if (fNumer >= fDenom) 
                    {
                        fS = 1;
                        fT = 0;
                        fSqrDist = fA00 + 2 * fB0 + fC;
                    }
                    else 
                    {
                        fS = fNumer / fDenom;
                        fT = 1 - fS;
                        fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                    }
                }
                else 
                {
                    fS = 0;
                    if (fTmp1 <= 0) 
                    {
                        fT = 1;
                        fSqrDist = fA11 + 2 * fB1 + fC;
                    }
                    else if (fB1 >= 0) 
                    {
                        fT = 0;
                        fSqrDist = fC;
                    }
                    else 
                    {
                        fT = -fB1 / fA11;
                        fSqrDist = fB1 * fT + fC;
                    }
                }
            }
            else if (fT < 0)   // region 6  
            {
                fTmp0 = fA01 + fB1;
                fTmp1 = fA00 + fB0;
                if (fTmp1 > fTmp0) 
                {
                    fNumer = fTmp1 - fTmp0;
                    fDenom = fA00 - 2 * fA01 + fA11;
                    if (fNumer >= fDenom) 
                    {
                        fT = 1;
                        fS = 0;
                        fSqrDist = fA11 + 2 * fB1 + fC;
                    }
                    else 
                    {
                        fT = fNumer / fDenom;
                        fS = 1 - fT;
                        fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                    }
                }
                else 
                {
                    fT = 0;
                    if (fTmp1 <= 0) 
                    {
                        fS = 1;
                        fSqrDist = fA00 + 2 * fB0 + fC;
                    }
                    else if (fB0 >= 0) 
                    {
                        fS = 0;
                        fSqrDist = fC;
                    }
                    else 
                    {
                        fS = -fB0 / fA00;
                        fSqrDist = fB0 * fS + fC;
                    }
                }
            }
            // region 1
            else 
            {
                fNumer = fA11 + fB1 - fA01 - fB0;
                if (fNumer <= 0) 
                {
                    fS = 0;
                    fT = 1;
                    fSqrDist = fA11 + 2 * fB1 + fC;
                }
                else 
                {
                    fDenom = fA00 - 2 * fA01 + fA11;
                    if (fNumer >= fDenom) 
                    {
                        fS = 1;
                        fT = 0;
                        fSqrDist = fA00 + 2 * fB0 + fC;
                    }
                    else 
                    {
                        fS = fNumer / fDenom;
                        fT = 1 - fS;
                        fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                    }
                }
            }
        }
        out[0] = fS;
        out[1] = fT;
        
        return Math.abs(fSqrDist);
    }
}
