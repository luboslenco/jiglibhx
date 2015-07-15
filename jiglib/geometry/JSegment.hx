package jiglib.geometry;

import jiglib.data.CollOutData;
import jiglib.math.JNumber3D;
import jiglib.math.JMath3D;
import jiglib.math.Vector3D;
import jiglib.physics.PhysicsState;

// A Segment is a line that starts at origin and goes only as far as (origin + delta).



class JSegment
{
    
    public var origin : Vector3D;
    public var delta : Vector3D;
    
    public function new(_origin : Vector3D, _delta : Vector3D)
    {
        this.origin = _origin;
        this.delta = _delta;
    }
    
    public function getPoint(t : Float) : Vector3D
    {
        return origin.add(JNumber3D.getScaleVector(delta, t));
    }
    
    public function getEnd() : Vector3D
    {
        return origin.add(delta);
    }
    
    public function clone() : JSegment
    {
        return new JSegment(origin, delta);
    }
    
    public function segmentSegmentDistanceSq(out : Array<Float>, seg : JSegment) : Float
    {
        var fA00 : Float;
        var fA01 : Float;
        var fA11 : Float;
        var fB0 : Float;
        var fC : Float;
        var fDet : Float;
        var fB1 : Float;
        var fS : Float;
        var fT : Float;
        var fSqrDist : Float;
        var fTmp : Float;
        var fInvDet : Float;
        
        var kDiff : Vector3D = origin.subtract(seg.origin);
        fA00 = delta.lengthSquared;
        fA01 = -delta.dotProduct(seg.delta);
        fA11 = seg.delta.lengthSquared;
        fB0 = kDiff.dotProduct(delta);
        fC = kDiff.lengthSquared;
        fDet = Math.abs(fA00 * fA11 - fA01 * fA01);
        
        if (fDet >= JMath3D.NUM_TINY) 
        {
            fB1 = -kDiff.dotProduct(seg.delta);
            fS = fA01 * fB1 - fA11 * fB0;
            fT = fA01 * fB0 - fA00 * fB1;
            
            if (fS >= 0) 
            {
                if (fS <= fDet) 
                {
                    if (fT >= 0) 
                    {
                        if (fT <= fDet) 
                        {
                            fInvDet = 1 / fDet;
                            fS *= fInvDet;
                            fT *= fInvDet;
                            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                        }
                        else 
                        {
                            fT = 1;
                            fTmp = fA01 + fB0;
                            if (fTmp >= 0) 
                            {
                                fS = 0;
                                fSqrDist = fA11 + 2 * fB1 + fC;
                            }
                            else if (-fTmp >= fA00) 
                            {
                                fS = 1;
                                fSqrDist = fA00 + fA11 + fC + 2 * (fB1 + fTmp);
                            }
                            else 
                            {
                                fS = -fTmp / fA00;
                                fSqrDist = fTmp * fS + fA11 + 2 * fB1 + fC;
                            }
                        }
                    }
                    else 
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
                }
                else 
                {
                    if (fT >= 0) 
                    {
                        if (fT <= fDet) 
                        {
                            fS = 1;
                            fTmp = fA01 + fB1;
                            if (fTmp >= 0) 
                            {
                                fT = 0;
                                fSqrDist = fA00 + 2 * fB0 + fC;
                            }
                            else if (-fTmp >= fA11) 
                            {
                                fT = 1;
                                fSqrDist = fA00 + fA11 + fC + 2 * (fB0 + fTmp);
                            }
                            else 
                            {
                                fT = -fTmp / fA11;
                                fSqrDist = fTmp * fT + fA00 + 2 * fB0 + fC;
                            }
                        }
                        else 
                        {
                            fTmp = fA01 + fB0;
                            if (-fTmp <= fA00) 
                            {
                                fT = 1;
                                if (fTmp >= 0) 
                                {
                                    fS = 0;
                                    fSqrDist = fA11 + 2 * fB1 + fC;
                                }
                                else 
                                {
                                    fS = -fTmp / fA00;
                                    fSqrDist = fTmp * fS + fA11 + 2 * fB1 + fC;
                                }
                            }
                            else 
                            {
                                fS = 1;
                                fTmp = fA01 + fB1;
                                if (fTmp >= 0) 
                                {
                                    fT = 0;
                                    fSqrDist = fA00 + 2 * fB0 + fC;
                                }
                                else if (-fTmp >= fA11) 
                                {
                                    fT = 1;
                                    fSqrDist = fA00 + fA11 + fC + 2 * (fB0 + fTmp);
                                }
                                else 
                                {
                                    fT = -fTmp / fA11;
                                    fSqrDist = fTmp * fT + fA00 + 2 * fB0 + fC;
                                }
                            }
                        }
                    }
                    else 
                    {
                        if (-fB0 < fA00) 
                        {
                            fT = 0;
                            if (fB0 >= 0) 
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
                        else 
                        {
                            fS = 1;
                            fTmp = fA01 + fB1;
                            if (fTmp >= 0) 
                            {
                                fT = 0;
                                fSqrDist = fA00 + 2 * fB0 + fC;
                            }
                            else if (-fTmp >= fA11) 
                            {
                                fT = 1;
                                fSqrDist = fA00 + fA11 + fC + 2 * (fB0 + fTmp);
                            }
                            else 
                            {
                                fT = -fTmp / fA11;
                                fSqrDist = fTmp * fT + fA00 + 2 * fB0 + fC;
                            }
                        }
                    }
                }
            }
            else 
            {
                if (fT >= 0) 
                {
                    if (fT <= fDet) 
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
                    else 
                    {
                        fTmp = fA01 + fB0;
                        if (fTmp < 0) 
                        {
                            fT = 1;
                            if (-fTmp >= fA00) 
                            {
                                fS = 1;
                                fSqrDist = fA00 + fA11 + fC + 2 * (fB1 + fTmp);
                            }
                            else 
                            {
                                fS = -fTmp / fA00;
                                fSqrDist = fTmp * fS + fA11 + 2 * fB1 + fC;
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
                }
                else 
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
            }
        }
        else 
        {
            if (fA01 > 0) 
            {
                if (fB0 >= 0) 
                {
                    fS = 0;
                    fT = 0;
                    fSqrDist = fC;
                }
                else if (-fB0 <= fA00) 
                {
                    fS = -fB0 / fA00;
                    fT = 0;
                    fSqrDist = fB0 * fS + fC;
                }
                else 
                {
                    fB1 = -kDiff.dotProduct(seg.delta);
                    fS = 1;
                    fTmp = fA00 + fB0;
                    if (-fTmp >= fA01) 
                    {
                        fT = 1;
                        fSqrDist = fA00 + fA11 + fC + 2 * (fA01 + fB0 + fB1);
                    }
                    else 
                    {
                        fT = -fTmp / fA01;
                        fSqrDist = fA00 + 2 * fB0 + fC + fT * (fA11 * fT + 2 * (fA01 + fB1));
                    }
                }
            }
            else 
            {
                if (-fB0 >= fA00) 
                {
                    fS = 1;
                    fT = 0;
                    fSqrDist = fA00 + 2 * fB0 + fC;
                }
                else if (fB0 <= 0) 
                {
                    fS = -fB0 / fA00;
                    fT = 0;
                    fSqrDist = fB0 * fS + fC;
                }
                else 
                {
                    fB1 = -kDiff.dotProduct(seg.delta);
                    fS = 0;
                    if (fB0 >= -fA01) 
                    {
                        fT = 1;
                        fSqrDist = fA11 + 2 * fB1 + fC;
                    }
                    else 
                    {
                        fT = -fB0 / fA01;
                        fSqrDist = fC + fT * (2 * fB1 + fA11 * fT);
                    }
                }
            }
        }
        
        out[0] = fS;
        out[1] = fT;
        return Math.abs(fSqrDist);
    }
    
    public function pointSegmentDistanceSq(out : Array<Float>, pt : Vector3D) : Float
    {
        var kDiff : Vector3D = pt.subtract(origin);
        var fT : Float = kDiff.dotProduct(delta);
        
        if (fT <= 0) 
        {
            fT = 0;
        }
        else 
        {
            var fSqrLen : Float = delta.lengthSquared;
            if (fT >= fSqrLen) 
            {
                fT = 1;
                kDiff = kDiff.subtract(delta);
            }
            else 
            {
                fT /= fSqrLen;
                kDiff = kDiff.subtract(JNumber3D.getScaleVector(delta, fT));
            }
        }
        
        out[0] = fT;
        return kDiff.lengthSquared;
    }
    
    public function segmentBoxDistanceSq(out : Array<Float>, rkBox : JBox, boxState : PhysicsState) : Float
    {
        out[3] = 0;
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        
        var obj : Array<Float> = [0, 0, 0, 0];
        var kRay : JRay = new JRay(origin, delta);
        var fSqrDistance : Float = sqrDistanceLine(obj, kRay, rkBox, boxState);
        if (obj[3] >= 0) 
        {
            if (obj[3] <= 1) 
            {
                out[3] = obj[3];
                out[0] = obj[0];
                out[1] = obj[1];
                out[2] = obj[2];
                return Math.max(fSqrDistance, 0);
            }
            else 
            {
                fSqrDistance = sqrDistancePoint(out, origin.add(delta), rkBox, boxState);
                out[3] = 1;
                return Math.max(fSqrDistance, 0);
            }
        }
        else 
        {
            fSqrDistance = sqrDistancePoint(out, origin, rkBox, boxState);
            out[3] = 0;
            return Math.max(fSqrDistance, 0);
        }
    }
    
    private function sqrDistanceLine(out : Array<Float>, rkLine : JRay, rkBox : JBox, boxState : PhysicsState) : Float
    {
        var kDiff : Vector3D;
        var kPnt : Vector3D;
        var kDir : Vector3D;
        var orientationCols : Array<Vector3D> = boxState.getOrientationCols();
        out[3] = 0;
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        
        kDiff = rkLine.origin.subtract(boxState.position);
        kPnt = new Vector3D(kDiff.dotProduct(orientationCols[0]), 
                kDiff.dotProduct(orientationCols[1]), 
                kDiff.dotProduct(orientationCols[2]));
        
        kDir = new Vector3D(rkLine.dir.dotProduct(orientationCols[0]), 
                rkLine.dir.dotProduct(orientationCols[1]), 
                rkLine.dir.dotProduct(orientationCols[2]));
        
        var kPntArr : Array<Float> = JNumber3D.toArray(kPnt);
        var kDirArr : Array<Float> = JNumber3D.toArray(kDir);
        
        var bReflect : Array<Bool> = [false, false, false];
        for (i in 0...3){
            if (kDirArr[i] < 0) 
            {
                kPntArr[i] = -kPntArr[i];
                kDirArr[i] = -kDirArr[i];
                bReflect[i] = true;
            }
            else 
            {
                bReflect[i] = false;
            }
        }
        
        JNumber3D.copyFromArray(kPnt, kPntArr);
        JNumber3D.copyFromArray(kDir, kDirArr);
        
        var obj : SegmentInfo = new SegmentInfo(kPnt.clone(), 0, 0);
        
        if (kDir.x > 0) 
        {
            if (kDir.y > 0) 
            {
                if (kDir.z > 0) 
                {
                    caseNoZeros(obj, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
                else 
                {
                    case0(obj, 0, 1, 2, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
            }
            else 
            {
                if (kDir.z > 0) 
                {
                    case0(obj, 0, 2, 1, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
                else 
                {
                    case00(obj, 0, 1, 2, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
            }
        }
        else 
        {
            if (kDir.y > 0) 
            {
                if (kDir.z > 0) 
                {
                    case0(obj, 1, 2, 0, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
                else 
                {
                    case00(obj, 1, 0, 2, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
            }
            else 
            {
                if (kDir.z > 0) 
                {
                    case00(obj, 2, 0, 1, kDir, rkBox);
                    out[3] = obj.pfLParam;
                }
                else 
                {
                    case000(obj, rkBox);
                    out[3] = 0;
                }
            }
        }
        
        kPntArr = JNumber3D.toArray(obj.rkPnt);
        for (i in 0...3){
            if (bReflect[i]) 
                kPntArr[i] = -kPntArr[i];
        }
        JNumber3D.copyFromArray(obj.rkPnt, kPntArr);
        
        out[0] = obj.rkPnt.x;
        out[1] = obj.rkPnt.y;
        out[2] = obj.rkPnt.z;
        
        return Math.max(obj.rfSqrDistance, 0);
    }
    
    private function sqrDistancePoint(out : Array<Float>, rkPoint : Vector3D, rkBox : JBox, boxState : PhysicsState) : Float
    {
        var kDiff : Vector3D;
        var kClosest : Vector3D;
        var boxHalfSide : Vector3D;
        var fSqrDistance : Float = 0;
        var fDelta : Float;
        
        var orientationVector : Array<Vector3D> = boxState.getOrientationCols();
        kDiff = rkPoint.subtract(boxState.position);
        kClosest = new Vector3D(kDiff.dotProduct(orientationVector[0]), 
                kDiff.dotProduct(orientationVector[1]), 
                kDiff.dotProduct(orientationVector[2]));
        
        boxHalfSide = rkBox.getHalfSideLengths();
        
        if (kClosest.x < -boxHalfSide.x) 
        {
            fDelta = kClosest.x + boxHalfSide.x;
            fSqrDistance += (fDelta * fDelta);
            kClosest.x = -boxHalfSide.x;
        }
        else if (kClosest.x > boxHalfSide.x) 
        {
            fDelta = kClosest.x - boxHalfSide.x;
            fSqrDistance += (fDelta * fDelta);
            kClosest.x = boxHalfSide.x;
        }
        
        if (kClosest.y < -boxHalfSide.y) 
        {
            fDelta = kClosest.y + boxHalfSide.y;
            fSqrDistance += (fDelta * fDelta);
            kClosest.y = -boxHalfSide.y;
        }
        else if (kClosest.y > boxHalfSide.y) 
        {
            fDelta = kClosest.y - boxHalfSide.y;
            fSqrDistance += (fDelta * fDelta);
            kClosest.y = boxHalfSide.y;
        }
        
        if (kClosest.z < -boxHalfSide.z) 
        {
            fDelta = kClosest.z + boxHalfSide.z;
            fSqrDistance += (fDelta * fDelta);
            kClosest.z = -boxHalfSide.z;
        }
        else if (kClosest.z > boxHalfSide.z) 
        {
            fDelta = kClosest.z - boxHalfSide.z;
            fSqrDistance += (fDelta * fDelta);
            kClosest.z = boxHalfSide.z;
        }
        
        out[0] = kClosest.x;
        out[1] = kClosest.y;
        out[2] = kClosest.z;
        
        return Math.max(fSqrDistance, 0);
    }
    
    private function face(out : SegmentInfo, i0 : Int, i1 : Int, i2 : Int, rkDir : Vector3D, rkBox : JBox, rkPmE : Vector3D) : Void
    {
        
        var fLSqr : Float;
        var fInv : Float;
        var fTmp : Float;
        var fParam : Float;
        var fT : Float;
        var fDelta : Float;
        
        var kPpE : Vector3D = new Vector3D();
        var boxHalfSide : Vector3D = rkBox.getHalfSideLengths();
        
        var boxHalfArr : Array<Float>;
        var rkPntArr : Array<Float>;
        var rkDirArr : Array<Float>;
        var kPpEArr : Array<Float>;
        var rkPmEArr : Array<Float>;
        boxHalfArr = JNumber3D.toArray(boxHalfSide);
        rkPntArr = JNumber3D.toArray(out.rkPnt);
        rkDirArr = JNumber3D.toArray(rkDir);
        kPpEArr = JNumber3D.toArray(kPpE);
        rkPmEArr = JNumber3D.toArray(rkPmE);
        
        kPpEArr[i1] = rkPntArr[i1] + boxHalfArr[i1];
        kPpEArr[i2] = rkPntArr[i2] + boxHalfArr[i2];
        JNumber3D.copyFromArray(rkPmE, kPpEArr);
        
        if (rkDirArr[i0] * kPpEArr[i1] >= rkDirArr[i1] * rkPmEArr[i0]) 
        {
            if (rkDirArr[i0] * kPpEArr[i2] >= rkDirArr[i2] * rkPmEArr[i0]) 
            {
                rkPntArr[i0] = boxHalfArr[i0];
                fInv = 1 / rkDirArr[i0];
                rkPntArr[i1] -= (rkDirArr[i1] * rkPmEArr[i0] * fInv);
                rkPntArr[i2] -= (rkDirArr[i2] * rkPmEArr[i0] * fInv);
                out.pfLParam = -rkPmEArr[i0] * fInv;
                JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
            }
            else 
            {
                fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i2] * rkDirArr[i2];
                fTmp = fLSqr * kPpEArr[i1] - rkDirArr[i1] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i2] * kPpEArr[i2]);
                if (fTmp <= 2 * fLSqr * boxHalfArr[i1]) 
                {
                    fT = fTmp / fLSqr;
                    fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
                    fTmp = kPpEArr[i1] - fT;
                    fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * fTmp + rkDirArr[i2] * kPpEArr[i2];
                    fParam = -fDelta / fLSqr;
                    out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + fTmp * fTmp + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);
                    
                    out.pfLParam = fParam;
                    rkPntArr[i0] = boxHalfArr[i0];
                    rkPntArr[i1] = fT - boxHalfArr[i1];
                    rkPntArr[i2] = -boxHalfArr[i2];
                    JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                }
                else 
                {
                    fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
                    fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * rkPmEArr[i1] + rkDirArr[i2] * kPpEArr[i2];
                    fParam = -fDelta / fLSqr;
                    out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + rkPmEArr[i1] * rkPmEArr[i1] + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);
                    
                    out.pfLParam = fParam;
                    rkPntArr[i0] = boxHalfArr[i0];
                    rkPntArr[i1] = boxHalfArr[i1];
                    rkPntArr[i2] = -boxHalfArr[i2];
                    JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                }
            }
        }
        else 
        {
            if (rkDirArr[i0] * kPpEArr[i2] >= rkDirArr[i2] * rkPmEArr[i0]) 
            {
                fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1];
                fTmp = fLSqr * kPpEArr[i2] - rkDirArr[i2] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1]);
                if (fTmp <= 2 * fLSqr * boxHalfArr[i2]) 
                {
                    fT = fTmp / fLSqr;
                    fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
                    fTmp = kPpEArr[i2] - fT;
                    fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * fTmp;
                    fParam = -fDelta / fLSqr;
                    out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + fTmp * fTmp + fDelta * fParam);
                    
                    out.pfLParam = fParam;
                    rkPntArr[i0] = boxHalfArr[i0];
                    rkPntArr[i1] = -boxHalfArr[i1];
                    rkPntArr[i2] = fT - boxHalfArr[i2];
                    JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                }
                else 
                {
                    fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
                    fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * rkPmEArr[i2];
                    fParam = -fDelta / fLSqr;
                    out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + rkPmEArr[i2] * rkPmEArr[i2] + fDelta * fParam);
                    
                    out.pfLParam = fParam;
                    rkPntArr[i0] = boxHalfArr[i0];
                    rkPntArr[i1] = -boxHalfArr[i1];
                    rkPntArr[i2] = boxHalfArr[i2];
                    JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                }
            }
            else 
            {
                fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i2] * rkDirArr[i2];
                fTmp = fLSqr * kPpEArr[i1] - rkDirArr[i1] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i2] * kPpEArr[i2]);
                if (fTmp >= 0) 
                {
                    if (fTmp <= 2 * fLSqr * boxHalfArr[i1]) 
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
                        fTmp = kPpEArr[i1] - fT;
                        fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * fTmp + rkDirArr[i2] * kPpEArr[i2];
                        fParam = -fDelta / fLSqr;
                        out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + fTmp * fTmp + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);
                        
                        out.pfLParam = fParam;
                        rkPntArr[i0] = boxHalfArr[i0];
                        rkPntArr[i1] = fT - boxHalfArr[i1];
                        rkPntArr[i2] = -boxHalfArr[i2];
                        JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                    }
                    else 
                    {
                        fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
                        fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * rkPmEArr[i1] + rkDirArr[i2] * kPpEArr[i2];
                        fParam = -fDelta / fLSqr;
                        out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + rkPmEArr[i1] * rkPmEArr[i1] + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);
                        
                        out.pfLParam = fParam;
                        rkPntArr[i0] = boxHalfArr[i0];
                        rkPntArr[i1] = boxHalfArr[i1];
                        rkPntArr[i2] = -boxHalfArr[i2];
                        JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                    }
                    return;
                }
                
                fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1];
                fTmp = fLSqr * kPpEArr[i2] - rkDirArr[i2] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1]);
                if (fTmp >= 0) 
                {
                    if (fTmp <= 2 * fLSqr * boxHalfArr[i2]) 
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
                        fTmp = kPpEArr[i2] - fT;
                        fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * fTmp;
                        fParam = -fDelta / fLSqr;
                        out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + fTmp * fTmp + fDelta * fParam);
                        
                        out.pfLParam = fParam;
                        rkPntArr[i0] = boxHalfArr[i0];
                        rkPntArr[i1] = -boxHalfArr[i1];
                        rkPntArr[i2] = fT - boxHalfArr[i2];
                        JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                    }
                    else 
                    {
                        fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
                        fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * rkPmEArr[i2];
                        fParam = -fDelta / fLSqr;
                        out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + rkPmEArr[i2] * rkPmEArr[i2] + fDelta * fParam);
                        
                        out.pfLParam = fParam;
                        rkPntArr[i0] = boxHalfArr[i0];
                        rkPntArr[i1] = -boxHalfArr[i1];
                        rkPntArr[i2] = boxHalfArr[i2];
                        JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
                    }
                    return;
                }
                
                fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
                fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * kPpEArr[i2];
                fParam = -fDelta / fLSqr;
                out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);
                
                out.pfLParam = fParam;
                rkPntArr[i0] = boxHalfArr[i0];
                rkPntArr[i1] = -boxHalfArr[i1];
                rkPntArr[i2] = -boxHalfArr[i2];
                JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
            }
        }
    }
    
    private function caseNoZeros(out : SegmentInfo, rkDir : Vector3D, rkBox : JBox) : Void
    {
        var boxHalfSide : Vector3D = rkBox.getHalfSideLengths();
        var kPmE : Vector3D = new Vector3D(out.rkPnt.x - boxHalfSide.x, out.rkPnt.y - boxHalfSide.y, out.rkPnt.z - boxHalfSide.z);
        
        var fProdDxPy : Float = rkDir.x * kPmE.y;
        var fProdDyPx : Float = rkDir.y * kPmE.x;
        var fProdDzPx : Float;
        var fProdDxPz : Float;
        var fProdDzPy : Float;
        var fProdDyPz : Float;
        
        if (fProdDyPx >= fProdDxPy) 
        {
            fProdDzPx = rkDir.z * kPmE.x;
            fProdDxPz = rkDir.x * kPmE.z;
            if (fProdDzPx >= fProdDxPz) 
            {
                face(out, 0, 1, 2, rkDir, rkBox, kPmE);
            }
            else 
            {
                face(out, 2, 0, 1, rkDir, rkBox, kPmE);
            }
        }
        else 
        {
            fProdDzPy = rkDir.z * kPmE.y;
            fProdDyPz = rkDir.y * kPmE.z;
            if (fProdDzPy >= fProdDyPz) 
            {
                face(out, 1, 2, 0, rkDir, rkBox, kPmE);
            }
            else 
            {
                face(out, 2, 0, 1, rkDir, rkBox, kPmE);
            }
        }
    }
    
    private function case0(out : SegmentInfo, i0 : Int, i1 : Int, i2 : Int, rkDir : Vector3D, rkBox : JBox) : Void
    {
        var boxHalfSide : Vector3D = rkBox.getHalfSideLengths();
        var boxHalfArr : Array<Float>;
        var rkPntArr : Array<Float>;
        var rkDirArr : Array<Float>;
        boxHalfArr = JNumber3D.toArray(boxHalfSide);
        rkPntArr = JNumber3D.toArray(out.rkPnt);
        rkDirArr = JNumber3D.toArray(rkDir);
        
        var fPmE0 : Float = rkPntArr[i0] - boxHalfArr[i0];
        var fPmE1 : Float = rkPntArr[i1] - boxHalfArr[i1];
        var fProd0 : Float = rkDirArr[i1] * fPmE0;
        var fProd1 : Float = rkDirArr[i0] * fPmE1;
        var fDelta : Float;
        var fInvLSqr : Float;
        var fInv : Float;
        var fPpE1 : Float;
        var fPpE0 : Float;
        
        if (fProd0 >= fProd1) 
        {
            rkPntArr[i0] = boxHalfArr[i0];
            
            fPpE1 = rkPntArr[i1] + boxHalfArr[i1];
            fDelta = fProd0 - rkDirArr[i0] * fPpE1;
            if (fDelta >= 0) 
            {
                fInvLSqr = 1 / (rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1]);
                out.rfSqrDistance += (fDelta * fDelta * fInvLSqr);
                
                rkPntArr[i1] = -boxHalfArr[i1];
                out.pfLParam = -(rkDirArr[i0] * fPmE0 + rkDirArr[i1] * fPpE1) * fInvLSqr;
            }
            else 
            {
                fInv = 1 / rkDirArr[i0];
                rkPntArr[i1] -= (fProd0 * fInv);
                out.pfLParam = -fPmE0 * fInv;
            }
            JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
        }
        else 
        {
            rkPntArr[i1] = boxHalfArr[i1];
            
            fPpE0 = rkPntArr[i0] + boxHalfArr[i0];
            fDelta = fProd1 - rkDirArr[i1] * fPpE0;
            if (fDelta >= 0) 
            {
                fInvLSqr = 1 / (rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1]);
                out.rfSqrDistance += (fDelta * fDelta * fInvLSqr);
                
                rkPntArr[i0] = -boxHalfArr[i0];
                out.pfLParam = -(rkDirArr[i0] * fPpE0 + rkDirArr[i1] * fPmE1) * fInvLSqr;
            }
            else 
            {
                fInv = 1 / rkDirArr[i1];
                rkPntArr[i0] -= (fProd1 * fInv);
                out.pfLParam = -fPmE1 * fInv;
            }
            JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
        }
        
        if (rkPntArr[i2] < -boxHalfArr[i2]) 
        {
            fDelta = rkPntArr[i2] + boxHalfArr[i2];
            out.rfSqrDistance += (fDelta * fDelta);
            rkPntArr[i2] = -boxHalfArr[i2];
        }
        else if (rkPntArr[i2] > boxHalfArr[i2]) 
        {
            fDelta = rkPntArr[i2] - boxHalfArr[i2];
            out.rfSqrDistance += (fDelta * fDelta);
            rkPntArr[i2] = boxHalfArr[i2];
        }
        JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
    }
    
    private function case00(out : SegmentInfo, i0 : Int, i1 : Int, i2 : Int, rkDir : Vector3D, rkBox : JBox) : Void
    {
        var fDelta : Float = 0;
        var boxHalfSide : Vector3D = rkBox.getHalfSideLengths();
        
        var boxHalfArr : Array<Float>;
        var rkPntArr : Array<Float>;
        var rkDirArr : Array<Float>;
        boxHalfArr = JNumber3D.toArray(boxHalfSide);
        rkPntArr = JNumber3D.toArray(out.rkPnt);
        rkDirArr = JNumber3D.toArray(rkDir);
        out.pfLParam = (boxHalfArr[i0] - rkPntArr[i0]) / rkDirArr[i0];
        
        rkPntArr[i0] = boxHalfArr[i0];
        
        if (rkPntArr[i1] < -boxHalfArr[i1]) 
        {
            fDelta = rkPntArr[i1] + boxHalfArr[i1];
            out.rfSqrDistance += (fDelta * fDelta);
            rkPntArr[i1] = -boxHalfArr[i1];
        }
        else if (rkPntArr[i1] > boxHalfArr[i1]) 
        {
            fDelta = rkPntArr[i1] - boxHalfArr[i1];
            out.rfSqrDistance += (fDelta * fDelta);
            rkPntArr[i1] = boxHalfArr[i1];
        }
        
        if (rkPntArr[i2] < -boxHalfArr[i2]) 
        {
            fDelta = rkPntArr[i2] + boxHalfArr[i2];
            out.rfSqrDistance += (fDelta * fDelta);
            rkPntArr[i2] = -boxHalfArr[i2];
        }
        else if (rkPntArr[i2] > boxHalfArr[i2]) 
        {
            fDelta = rkPntArr[i2] - boxHalfArr[i2];
            out.rfSqrDistance += (fDelta * fDelta);
            rkPntArr[i2] = boxHalfArr[i2];
        }
        
        JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
    }
    
    private function case000(out : SegmentInfo, rkBox : JBox) : Void
    {
        var fDelta : Float = 0;
        var boxHalfSide : Vector3D = rkBox.getHalfSideLengths();
        
        if (out.rkPnt.x < -boxHalfSide.x) 
        {
            fDelta = out.rkPnt.x + boxHalfSide.x;
            out.rfSqrDistance += (fDelta * fDelta);
            out.rkPnt.x = -boxHalfSide.x;
        }
        else if (out.rkPnt.x > boxHalfSide.x) 
        {
            fDelta = out.rkPnt.x - boxHalfSide.x;
            out.rfSqrDistance += (fDelta * fDelta);
            out.rkPnt.x = boxHalfSide.x;
        }
        
        if (out.rkPnt.y < -boxHalfSide.y) 
        {
            fDelta = out.rkPnt.y + boxHalfSide.y;
            out.rfSqrDistance += (fDelta * fDelta);
            out.rkPnt.y = -boxHalfSide.y;
        }
        else if (out.rkPnt.y > boxHalfSide.y) 
        {
            fDelta = out.rkPnt.y - boxHalfSide.y;
            out.rfSqrDistance += (fDelta * fDelta);
            out.rkPnt.y = boxHalfSide.y;
        }
        
        if (out.rkPnt.z < -boxHalfSide.z) 
        {
            fDelta = out.rkPnt.z + boxHalfSide.z;
            out.rfSqrDistance += (fDelta * fDelta);
            out.rkPnt.z = -boxHalfSide.z;
        }
        else if (out.rkPnt.z > boxHalfSide.z) 
        {
            fDelta = out.rkPnt.z - boxHalfSide.z;
            out.rfSqrDistance += (fDelta * fDelta);
            out.rkPnt.z = boxHalfSide.z;
        }
    }
}


class SegmentInfo
{
    public var rkPnt : Vector3D;
    public var pfLParam : Float;
    public var rfSqrDistance : Float;
    
    @:allow(jiglib.geometry)
    private function new(rkPnt : Vector3D = null, pfLParam : Float = 0, rfSqrDistance : Float = 0)
    {
        this.rkPnt = (rkPnt != null) ? rkPnt : new Vector3D();
        this.pfLParam = (Math.isNaN(pfLParam)) ? 0 : pfLParam;
        this.rfSqrDistance = (Math.isNaN(rfSqrDistance)) ? 0 : rfSqrDistance;
    }
}
