package jiglib.math;

class JMatrix3D
{
    public static function getTranslationMatrix(x : Float, y : Float, z : Float) : Matrix3D
    {
        var matrix3D : Matrix3D = new Matrix3D();
        matrix3D.appendTranslation(x, y, z);
        return matrix3D;
    }
    
    public static function getScaleMatrix(x : Float, y : Float, z : Float) : Matrix3D
    {
        var matrix3D : Matrix3D = new Matrix3D();
        matrix3D.prependScale(x, y, z);
        return matrix3D;
    }
    
    public static function getRotationMatrix(x : Float, y : Float, z : Float, degree : Float, pivotPoint : Vector3D = null) : Matrix3D
    {
        var matrix3D : Matrix3D = new Matrix3D();
        matrix3D.appendRotation(degree, new Vector3D(x, y, z), pivotPoint);
        return matrix3D;
    }
    
    public static function getInverseMatrix(m : Matrix3D) : Matrix3D
    {
        var matrix3D : Matrix3D = m.clone();
        matrix3D.invert();
        return matrix3D;
    }
    
    public static function getTransposeMatrix(m : Matrix3D) : Matrix3D
    {
        var matrix3D : Matrix3D = m.clone();
        matrix3D.transpose();
        return matrix3D;
    }
    
    public static function getAppendMatrix3D(a : Matrix3D, b : Matrix3D) : Matrix3D
    {
        var matrix3D : Matrix3D = a.clone();
        matrix3D.append(b);
        return matrix3D;
    }
    
    public static function getPrependMatrix(a : Matrix3D, b : Matrix3D) : Matrix3D
    {
        var matrix3D : Matrix3D = a.clone();
        matrix3D.prepend(b);
        return matrix3D;
    }
    
    public static function getSubMatrix(a : Matrix3D, b : Matrix3D) : Matrix3D
    {
        var ar : Array<Float> = a.rawData;
        var br : Array<Float> = b.rawData;
        return new Matrix3D([
                ar[0] - br[0], 
                ar[1] - br[1], 
                ar[2] - br[2], 
                ar[3] - br[3], 
                ar[4] - br[4], 
                ar[5] - br[5], 
                ar[6] - br[6], 
                ar[7] - br[7], 
                ar[8] - br[8], 
                ar[9] - br[9], 
                ar[10] - br[10], 
                ar[11] - br[11], 
                ar[12] - br[12], 
                ar[13] - br[13], 
                ar[14] - br[14], 
                ar[15] - br[15]]);
    }
    
    public static function getRotationMatrixAxis(degree : Float, rotateAxis : Vector3D = null) : Matrix3D
    {
        var matrix3D : Matrix3D = new Matrix3D();
        matrix3D.appendRotation(degree, (rotateAxis != null) ? rotateAxis : Vector3D.X_AXIS);
        return matrix3D;
    }
    
    public static function getCols(matrix3D : Matrix3D) : Array<Vector3D>
    {
        var rawData : Array<Float> = matrix3D.rawData;
        var cols : Array<Vector3D> = new Array<Vector3D>();
        
        cols.push( new Vector3D(rawData[0], rawData[1], rawData[2]) );
        cols.push( new Vector3D(rawData[4], rawData[5], rawData[6]) );
        cols.push( new Vector3D(rawData[8], rawData[9], rawData[10]) );
        
        return cols;
    }
    
    public static function multiplyVector(matrix3D : Matrix3D, v : Vector3D) : Void
    {
        v = matrix3D.transformVector(v);
    }
}
