package;

import h3d.col.Point;
import h3d.prim.Polygon;
import h3d.prim.UV;

using StringTools;

private class OBJGroup
{
	public var name:String;
	public var points:Array<Point>;
	public var uvs:Array<UV>;
	public var normals:Array<Point>;

	public function new(_name:String)
	{
		name = _name;
		points = [];
		uvs = [];
		normals = [];
	}

	public function isEmpty():Bool
	{
		return points.length == 0 && uvs.length == 0 && normals.length == 0;
	}
}

class HeapsOBJModel
{
	public var objects:Array<Polygon>;

	public function new()
	{
		objects = [];
	}

	public static function load(data:String):HeapsOBJModel
	{
		var groups:Array<OBJGroup> = [];
		var points:Array<Point> = [];
		var uvs:Array<UV> = [];
		var normals:Array<Point> = [];

		var currentGroup = new OBJGroup("default");
		var lines = data.split("\n").map(cleanLine);
		for (line in lines) {
			var params = line.split(" ").filter(isNonEmpty);
			switch (params[0]) {
				case "g": //NOTE: only supports one group name
					if (!currentGroup.isEmpty()) {
						groups.push(currentGroup);
					}
					var groupName = params[1];
					currentGroup = new OBJGroup(groupName);

				case "v":
					var x:Float = Std.parseFloat(params[1]);
					var y:Float = Std.parseFloat(params[2]);
					var z:Float = Std.parseFloat(params[3]);
					points.push( new Point(x, y, z) );

				case "vt":
					var u:Float = Std.parseFloat(params[1]);
					var v:Float = Std.parseFloat(params[2]);
					uvs.push( new UV(u, v) );

				case "vn":
					var x:Float = Std.parseFloat(params[1]);
					var y:Float = Std.parseFloat(params[2]);
					var z:Float = Std.parseFloat(params[3]);
					normals.push( new Point(x, y, z) );

				case "f":
					var verts:Array<String> = params.slice(1,4); //NOTE: only supports triangles
					verts.reverse();
					for (vert in verts) {
						var parts = vert.split("/");
						var hasUV = parts.length >= 2 && parts[1] != "";
						var hasNormal = parts.length >= 3 && parts[2] != "";
						//NOTE: unable to resolve references to vectors that have not been declared yet
						var point = points[Std.parseInt(parts[0]) - 1].clone();
						var uv = hasUV ? uvs[Std.parseInt(parts[1]) - 1].clone() : new UV(0, 0);
						var normal = hasNormal ? normals[Std.parseInt(parts[2]) - 1].clone() : new Point(1, 0, 0);

						point.z = -point.z;
						uv.v = 1-uv.v; //HACK: fix inverted v coord bug
						normal.z = -normal.z;

						currentGroup.points.push(point);
						currentGroup.uvs.push(uv);
						currentGroup.normals.push(normal);
					}

					//TODO: support for .mtl

				default:
			}
		}
		if (!currentGroup.isEmpty()) {
			groups.push(currentGroup);
		}

		var objModel = new HeapsOBJModel();
		objModel.objects = groups.map(function(g) {
			var poly = new Polygon(g.points);
			poly.uvs = g.uvs;
			poly.normals = g.normals;
			return poly;
		});
		return objModel;
	}

	private static function cleanLine(s:String):String
	{
		var trimmed = s.trim();
		var hashPos = s.indexOf("#");
		return (hashPos > -1) ? trimmed.substring(0, hashPos) : trimmed;
	}

	private static function isNonEmpty(s:String):Bool
	{
		return s.length > 0;
	}
}
