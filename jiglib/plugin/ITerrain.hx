package jiglib.plugin;


interface ITerrain
{
    
    //Min of coordinate horizontally;
    var minW(get, never) : Float;
    
    //Min of coordinate vertically;
    var minH(get, never) : Float;
    
    //Max of coordinate horizontally;
    var maxW(get, never) : Float;
    
    //Max of coordinate vertically;
    var maxH(get, never) : Float;
    
    //The horizontal length of each segment;
    var dw(get, never) : Float;
    
    //The vertical length of each segment;
    var dh(get, never) : Float;
    
    //Number of segments horizontally.
    var sw(get, never) : Int;
    
    //Number of segments vertically
    var sh(get, never) : Int;
    
    //the heights of all vertices
    var heights(get, never) : Array<Array<Float>>;
    
    var maxHeight(get, never) : Float;

}
