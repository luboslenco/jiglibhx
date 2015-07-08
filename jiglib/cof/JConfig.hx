package jiglib.cof;


class JConfig
{
    public static var solverType : String = "ACCUMULATED";  //the type of collision solve, allowable value: FAST,NORMAL,ACCUMULATED  
    public static var rotationType : String = "DEGREES";  // can be either RADIANS or DEGREES;  
    public static var doShockStep : Bool = false;  //if do a shock step to help stacking.  
    public static var allowedPenetration : Float = 0.01;  // How much penetration to allow  
    public static var collToll : Float = 0.05;  // the tolerance for collision detection  
    public static var velThreshold : Float = 0.5;  // indicates the line velocity threshold for freezing  
    public static var angVelThreshold : Float = 0.5;  // indicates the angle velocity threshold for freezing  
    public static var posThreshold : Float = 0.2;  // change for detecting position changes during deactivation  
    public static var orientThreshold : Float = 0.2;  // change for detecting orientation changes during deactivation.  
    public static var deactivationTime : Float = 0.5;  // how long it takes to go from active to frozen when stationary.  
    public static var numPenetrationRelaxationTimesteps : Int = 10;  // number of timesteps to resolve penetration over  
    public static var numCollisionIterations : Int = 1;  // number of collision iterations  
    public static var numContactIterations : Int = 2;  // number of contact iteratrions  
    public static var numConstraintIterations : Int = 2;  // number of Constraint iteratrions  
}
