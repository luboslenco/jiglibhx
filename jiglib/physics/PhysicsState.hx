package jiglib.physics;

import jiglib.math.*;

class PhysicsState
{
    public var position : Vector3D = new Vector3D();
    public var orientation : Matrix3D = new Matrix3D();
    public var linVelocity : Vector3D = new Vector3D();
    public var rotVelocity : Vector3D = new Vector3D();
    //public var orientationCols:Vector.<Vector3D> = new Vector.<Vector3D>(3,true);
    
    public function new()
    {
        //orientationCols[0] = new Vector3D();
        //orientationCols[1] = new Vector3D();
        //orientationCols[2] = new Vector3D();
        
    }
    /*
		public function get orientation():Matrix3D { return _orientation; }
		public function set orientation(val:Matrix3D):void 
		{ 
			_orientation = val;		
			var _rawData:Vector.<Number> = _orientation.rawData;
			
			orientationCols[0].x = _rawData[0];
			orientationCols[0].y = _rawData[1];
			orientationCols[0].z = _rawData[2];
			
			orientationCols[1].x = _rawData[4];
			orientationCols[1].y = _rawData[5];
			orientationCols[1].z = _rawData[6];
			
			orientationCols[2].x = _rawData[8];
			orientationCols[2].y = _rawData[9];
			orientationCols[2].z = _rawData[10];
		}
	*/
    //here for backwards compatibility should use public function instead unless you need a clone
    public function getOrientationCols() : Array<Vector3D>
    {
        return JMatrix3D.getCols(orientation);
    }
}
