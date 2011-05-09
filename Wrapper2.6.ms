-- Wrapper 2.6 by Rab Gordon for 3DS MAX / GMAX - www.cnc-toolkit.com

-- Credits;
-- Thanks to Swami Lama and Jared Keller for help on the Barycentric Coords stuff
-- uses snippets of functions by John Burnett for getting to the unwrapped mesh

-- WHAT IT DOES:
-- Wraps one object around another
-- Takes either a spline or a mesh (the WrapSource) and Wraps it to or around another object (the Target Surface), 
-- Uses either a projection onto the mesh or the UnWrapped UVW coordinates of the mesh to calculate the wrap around,
-- Source Mesh vertex heights above Z=0 are transformed to an offset distance from the Target Mesh
-- Can choose to align the source to the target mesh curvature or have a straight projection ( hard to explain, try it and see)

-- OPERATION: 
-- 1st define WrapSource and WrapTarget objects with the Pick butons,
--
-- Ensure that the WrapSource has its base at height Z=0 or position it above 0 to increase the offset
-- 
-- If using one of the 1st three methods (which project the WrapSource downwards) then position the WrapTarget beneath the WrapSource,
-- 
-- By choosing the UVW mapping option, an unwrapped version of the Target Surface is created, position this beneath the WrapSource
--
-- Press GO,  ( the operation could take a while on complex meshes )


Clearlistener()
if Debug == undefined then Global Debug = false
if WrapperFloater != undefined then(closerolloutfloater WrapperFloater)
Global WrapSource, WrapTarget, ShowNormals, UnwrappedMesh, Point3Pos, Normal, NewVertPos, tmpSpline

Global ClosestVertex
Global ClosestFace
Global triangleArea
Global ClosestPoint
Global getBCoords
Global Point3FromBaryCoords
Global NormalFromBaryCoords
Global WrapSpline
Global WrapMesh
Global UnwrapMesh
Global WrapMeshUVW
Global WrapSplineUVW
Global FaceCentre
Global getClosestBCoords 
Global NormalFromBaryCoords
Global NewVertPosOffsetByNormal


----------------------------
-- Debug Box - Creates a box a given position, helpfull for debugging
fn dbbox val = box length:3 width:3 height:3 pos:val
fn dbpyr val size = pyramid width:size depth:size height:size pos:val
----------------------------
-- Get the id number of the closest vertex to a point3 coordinate
fn ClosestVertex MeshObj coords =
( if Debug then format "ClosestVertex: MeshObj=% coords=% \n" MeshObj coords
VertexArray= #{1..MeshObj.numVerts} 
local closest = 999999999, closestIdx, dist
for i in VertexArray do 
	( dist=(distance coords (GetVert MeshObj i)) 
	if dist < closest then (closest = dist;	closestIdx = i)
	) 
if Debug then format "ClosestVertex:closestIdx = %\n" closestIdx
return closestIdx 
)
-----------------------------------------------------------------------------------------
-- Get the id number of the closest face to a point3 coordinate
fn ClosestFace MeshObj coords =	
(if Debug then format "ClosestFace: MeshObj= % coords= % numFaces= %\n" MeshObj coords (MeshObj.numFaces)
FaceArray= #{1..MeshObj.numFaces} 
local closest = 999999999, closestIdx, dist
for i in FaceArray do 
	( dist=(distance coords (FaceCentre MeshObj i)) 
	if dist < closest then (closest = dist;	closestIdx = i)
	) 
if Debug then format "ClosestFace:closestIdx = %\n" closestIdx
return closestIdx 	
)
-----------------------------------------------------------------------------------------
-- Get the Point3 coordinates of the centre of a face
fn FaceCentre MeshObj FaceNo =
(--if Debug then format "FaceCentre: MeshObj=% FaceNo=% \n" MeshObj FaceNo
local vf = getFace MeshObj FaceNo
local p0 = getVert MeshObj vf.x
local p1 = getVert MeshObj vf.y
local p2 = getVert MeshObj vf.z
local cx = (p0.x + p1.x + p2.x) / 3
local cy = (p0.y + p1.y + p2.y) / 3
local cz = (p0.z + p1.z + p2.z) / 3
local c = [cx,cy,cz]
return c
)
-----------------------------------------------------------------------------------------
-- Get the Area of a triangle given three points
fn triangleArea v0 v1 v2 =
(if Debug then format "triangleArea: v0=% v1=% v2=%\n" v0 v1 v2 
	local a=distance v0 v1
	local b=distance v1 v2
	local c=distance v2 v0
	local s=(a+b+c)/2.
	local theArea=sqrt(s*(s-a)*(s-b)*(s-c))
	return theArea
)
-----------------------------------------------------------------------------------------
-- Get the closest point on a mesh to a close point3 position given the id of the nearest face it lies on 
fn ClosestPoint MeshObj coords =
(if Debug then format "ClosestPoint: MeshObj=% coords=%\n" MeshObj coords
local FaceNo = ClosestFace MeshObj coords ; if Debug then format "ClosestPoint: FaceNo=%\n" FaceNo
local vf = getFace MeshObj FaceNo ; if Debug then format "ClosestPoint: vf=%\n" vf
local p0 = getVert MeshObj vf.x
local p1 = getVert MeshObj vf.y
local p2 = getVert MeshObj vf.z
--local BCoords = getBCoords coords p0 p1 p2
local BCoords = getClosestBCoords coords p0 p1 p2 
ClosestPointPos = Point3FromBaryCoords MeshObj FaceNo BCoords
if Debug then format "ClosestPoint =%\n" ClosestPointPos
return ClosestPointPos
)
-----------------------------------------------------------------------------------------
-- Get the Closest barycentric coordinates (b0, b1, b2) to a point (p) given the triangle vertex coordinates (p0, p1, p2)
fn getClosestBCoords p p0 p1 p2 =
(if Debug then format "\n getClosestBCoords: p=%, p0=%, p1=%, p2=%\n" p p0 p1 p2
	local d0 = distance p p0
	local d1 = distance p p1
	local d2 = distance p p2
	c = [d0,d1,d2]
	if Debug then format "c: %\n" c
	
	c.x = 1 / c.x
	c.y = 1 / c.y
	c.z = 1 / c.z
	if Debug then format "1/c: %\n" c
	
	c = c/(c.x+c.y+c.z)
	if Debug then format "c/(c.x+c.y+c.z): %\n" c	
	
	return c	
)
-----------------------------------------------------------------------------------------
-- Get the barycentric coordinates (b0, b1, b2) of a point (p) within a triangle, given the triangle vertex coordinates (p0, p1, p2) and the position of p.
fn getBCoords p p0 p1 p2 =
(if Debug then format "getBCoords: p=%, p0=%, p1=%, p2=%\n" p p0 p1 p2
	local areaFace=triangleArea p0 p1 p2
	local areab0=triangleArea p1 p2 p
	local areab1=triangleArea p2 p0 p
	local areab2=triangleArea p0 p1 p
	
	local b0=areab0/areaFace
	local b1=areab1/areaFace
	local b2=areab2/areaFace
	local b=[b0,b1,b2]	-- Barycentric coordinates.
	--local totalarea = (areab0 + areab1 + areab2)
	--b= [areab0/totalarea,areab1/totalarea,areab2/totalarea]
	format "Barycentric %\n" b
	return b
)
-------------------------------------------------------------------------------------------
-- Get Point3 coordinates of point on a mesh surface given the mesh, the face that the point lies on and the Barycentric Coordinates
fn Point3FromBaryCoords WrapTarget FaceNo BCoords =
(if Debug then format "Point3FromBaryCoords: WrapTarget=% FaceNo=% BCoords=%\n" WrapTarget FaceNo BCoords 
	local vf  = getFace WrapTarget FaceNo 
	local vv1 = getVert WrapTarget vf.x 
	local vv2 = getVert WrapTarget vf.y 
	local vv3 = getVert WrapTarget vf.z 
	return Point3Pos = vv1*BCoords.x + vv2*BCoords.y + vv3*BCoords.z 
)
-------------------------------------------------------------------------------------------
-- Get Normal vector of point on a mesh given the mesh, the face that the point lies on and the Barycentric Coordinates
fn NormalFromBaryCoords WrapTarget FaceNo BCoords =
(if Debug then format "NormalFromBaryCoords: WrapTarget=% FaceNo=% BCoords=%\n" WrapTarget FaceNo BCoords
	local vf  = getFace WrapTarget FaceNo 
	local vn1 = GetNormal WrapTarget vf.x
	local vn2 = GetNormal WrapTarget vf.y
	local vn3 = GetNormal WrapTarget vf.z
	local Normal = vn1*BCoords.x + vn2*BCoords.y + vn3*BCoords.z 
	return Normal
)
---------------------------------------------------------------------------------------
Function WrapSpline Method =
( if Debug then format "WrapSpline: Method= %\n" Method
Local Point3Pos = [0,0,0] , IntersectDir = [0,0,0], IntersectPos = [0,0,0]
if ShowNormals then NSpline = splineShape prefix: "Normal_Spline"
ProjectionTape= tape pos:[0,0,0] target:(targetObject pos:[0,0,0] )

for SplineNumber = 1 to NumSplines WrapSource do
	(
	for VertNumber = 1 to (numknots WrapSource SplineNumber) do
		(if Debug then format "\nVertNumber=%\n" VertNumber
		VertPosition = getknotpoint WrapSource SplineNumber VertNumber
		ProjectionTape.pos = VertPosition ; ProjectionTape.target.pos = VertPosition - [0,0,10]
		ir = intersectRay WrapTarget (ProjectionTape as ray)
							
		if ir != undefined then	---- if ray hit smesh
		( IntersectPos = ir.pos ; IntersectDir = ir.dir 
			case Method of 
			( 
			1:( Point3Pos = VertPosition; Point3Pos.z +=  IntersectPos.z ; Point3Pos = ( IntersectPos + ( IntersectDir * (Distance Point3Pos IntersectPos ))))
			2:( Point3Pos = VertPosition; Point3Pos.z +=  IntersectPos.z )
			3:( Point3Pos = IntersectPos )
			)	
			NewVertPos = Point3Pos 
		 	if Debug then format "NewVertPos =%\n" NewVertPos 
			setKnotPoint WrapSource SplineNumber VertNumber NewVertPos 
			setKnotType WrapSource SplineNumber VertNumber #Corner 
			
					if ShowNormals then 
					(  	tmpSpline = addNewSpline NSpline 
					--   	addKnot NSpline tmpSpline #corner #line IntersectPos 
						addKnot NSpline tmpSpline #corner #line VertPosition
						addKnot NSpline tmpSpline #corner #line NewVertPos 
					)

		)	
			else 
			(if Debug then format "Miss\n"
			IntersectPos = WrapTarget.min; IntersectDir = [0,0,0]
			NewVertPos = VertPosition
			if Method != 3 then NewVertPos.z += IntersectPos.z else NewVertPos.z = IntersectPos.z
			setKnotPoint WrapSource SplineNumber VertNumber NewVertPos
			setKnotType WrapSource SplineNumber VertNumber #Corner 
			)
		)
	)
try (updateShape NSpline) catch()
UpdateShape WrapSource
Delete ProjectionTape
RedrawViews()
)
------------------------------------------------------------------------------------------------------------------------------------------------------
Function WrapMesh Method =
( if Debug then format "WrapMesh: Method= %\n" Method
Local Point3Pos = [0,0,0] , IntersectDir = [0,0,0], IntersectPos = [0,0,0]
ProjectionTape= tape pos:[0,0,0] target:(targetObject pos:[0,0,0] )

for VertNumber = 1 to ( getNumVerts WrapSource ) do
	( 
	VertPosition = getVert WrapSource VertNumber
	ProjectionTape.pos = VertPosition ; ProjectionTape.target.pos = VertPosition - [0,0,10]
	ir = intersectRay WrapTarget (ProjectionTape as ray)
		
		if ir != undefined then	---- if ray hit smesh
		( IntersectPos = ir.pos ; IntersectDir = ir.dir 
			case Method of 
			( 
			1:( Point3Pos = VertPosition; Point3Pos.z +=  IntersectPos.z ; Point3Pos = ( IntersectPos + ( IntersectDir * (Distance Point3Pos IntersectPos ))))
			2:( Point3Pos = VertPosition; Point3Pos.z +=  IntersectPos.z )
			3:( Point3Pos = IntersectPos )
			)	
			setVert WrapSource VertNumber Point3Pos 
		)
	
		else 	---- if ray misses mesh
		( IntersectPos = WrapTarget.min; IntersectDir = [0,0,0]
			Point3Pos = VertPosition
			if Method != 3 then Point3Pos.z +=  IntersectPos.z	else Point3Pos.z = IntersectPos.z
			setVert WrapSource VertNumber Point3Pos
		)
	)
update WrapSource
Delete ProjectionTape
RedrawViews()
)
-----------------------------------------------------------------------------------------
Function UnwrapMesh WrapTarget=
(if Debug then format "UnwrapMesh: WrapTarget= %\n" WrapTarget
		try WrapTarget.mapcoords=true catch()
		WrapTarget = copy WrapTarget Name:("Unwrapped_WrapTarget_"+ WrapTarget.name)
		ConvertToMesh WrapTarget
		if not meshop.getMapSupport  WrapTarget 1 then 
		( AddModifier WrapTarget (UVWmap()); ConvertToMesh WrapTarget)

UnwrappedMesh = mesh name:(WrapTarget.name + "_UNWRAPPED") numVerts:(meshOp.GetNumMapVerts WrapTarget 1) numFaces:WrapTarget.numFaces

for i in 1 to UnwrappedMesh.numVerts do 
(
UnwrappedVert=(meshOp.GetMapVert WrapTarget 1 i)
UnwrappedVert.z = 0.0
setVert UnwrappedMesh i UnwrappedVert
)

for i in 1 to UnwrappedMesh.numFaces do 
(
SetFace UnwrappedMesh i (meshOp.GetMapFace WrapTarget 1 i) 
for j in 1 to 3 do (SetEdgeVis UnwrappedMesh i j (GetEdgeVis WrapTarget i j))
)
AddModifier UnwrappedMesh (UVWmap())
UnwrappedMesh.material = WrapTarget.material
ConvertToMesh UnwrappedMesh
Update UnwrappedMesh 
Delete WrapTarget
RedrawViews()
)
-----------------------------------------------------------------------------------------
Function WrapMeshUVW =
( if Debug then format "WrapMeshUVW: \n"
Global Point3Pos = [0,0,0] , IntersectDir = [0,0,0], IntersectPos = [0,0,0]
ProjectionTape= tape pos:[0,0,0] target:(targetObject pos:[0,0,0] )

if ShowNormals then NSpline = splineShape prefix: "Normal_Spline"

for VertNumber = 1 to getNumVerts WrapSource do
	(
		VertPosition = getVert WrapSource VertNumber
		ProjectionTape.pos = VertPosition + [0,0,10] ; ProjectionTape.target.pos = VertPosition - [0,0,10]
		irEX = intersectRayEX UnwrappedMesh (ProjectionTape as ray)
	
		if irEX != undefined then	
			(
			Offset = Distance VertPosition irEX[1].pos
			FaceNo = irEX[2]
			BCoords = irEX[3]
			Point3Pos=	Point3FromBaryCoords WrapTarget FaceNo BCoords
			Normal= NormalFromBaryCoords WrapTarget FaceNo BCoords
			NewVertPosOffsetByNormal = ( Point3Pos + ( Normal * Offset))
				
			if ShowNormals then 
					(   tmpSpline = addNewSpline NSpline 
			           	addKnot NSpline tmpSpline #corner #line Point3Pos
			           	addKnot NSpline tmpSpline #corner #line NewVertPosOffsetByNormal
					)
	     	
			setVert WrapSource VertNumber NewVertPosOffsetByNormal 
			)	
		
			else 
			(
			IntersectPos = WrapTarget.min; IntersectDir = [0,0,0]
			Point3Pos = VertPosition
			Point3Pos.z +=  IntersectPos.z
			setVert WrapSource VertNumber Point3Pos 
			)
		
	)
Update WrapSource
try (updateShape NSpline) catch()
Delete ProjectionTape
RedrawViews()
)
------------------------------------------------------------------------------------------
Function WrapSplineUVW =
( if Debug then format "WrapSplineUVW: \n"
Global Point3Pos = [0,0,0] , IntersectDir = [0,0,0], IntersectPos = [0,0,0]
ProjectionTape= tape pos:[0,0,0] target:(targetObject pos:[0,0,0] )

if ShowNormals then NSpline = splineShape prefix: "Normal_Spline"

for SplineNumber = 1 to NumSplines WrapSource do
	(NumSplines 
	for VertNumber = 1 to (numknots WrapSource SplineNumber) do
		(
		VertPosition = getknotpoint WrapSource SplineNumber VertNumber
		ProjectionTape.pos = VertPosition + [0,0,10] ; ProjectionTape.target.pos = VertPosition - [0,0,10]
		irEX = intersectRayEX UnwrappedMesh (ProjectionTape as ray)
	
		if irEX != undefined then	
			(
			Offset = Distance VertPosition irEX[1].pos
			FaceNo = irEX[2]
			BCoords = irEX[3]
			Point3Pos=	Point3FromBaryCoords WrapTarget FaceNo BCoords
			Normal= NormalFromBaryCoords WrapTarget FaceNo BCoords
			NewVertPosOffsetByNormal = ( Point3Pos + ( Normal * Offset))
				
			if ShowNormals then 
					(   tmpSpline = addNewSpline NSpline 
			           	addKnot NSpline tmpSpline #corner #line Point3Pos
			           	addKnot NSpline tmpSpline #corner #line NewVertPosOffsetByNormal
					)
	     	
			setKnotPoint WrapSource SplineNumber VertNumber NewVertPosOffsetByNormal 
	     	setKnotType WrapSource SplineNumber VertNumber #Corner 
			)	
		
			else 
			(
			IntersectPos = WrapTarget.min; IntersectDir = [0,0,0]
			Point3Pos = VertPosition
			Point3Pos.z +=  IntersectPos.z
			setKnotPoint WrapSource SplineNumber VertNumber Point3Pos
	     	setKnotType WrapSource SplineNumber VertNumber #Corner 
			)
		)
	)
UpdateShape WrapSource
try (updateShape NSpline) catch()
Delete ProjectionTape
RedrawViews()
)

------------------------------------------------------------------------------------------
WrapperFloater rollout WrapperRollout "Object Wrapper"
(
Group "Wrap Source"
(PickButton pbWrapSource 	"Pick" 	width:100 height:15 tooltip: "Pick Mesh or Spline")
Group "Wrap Target"
(PickButton pbWrapTarget "Pick"		width:100 height:15 tooltip: "Pick Target Mesh")
radiobuttons WrapMethod labels:#("Curve to Surface","Straight Projection","Conform to Surface","Use UVW Mapping") Default:1
checkbox cbShowNormals	"Show Normals" offset:[-6,-4]
Button bGO    	"GO" 		width:50 height:20 tooltip: "GO !"

on pbWrapSource	picked obj do pbWrapSource.text = obj.name
on pbWrapTarget		picked obj do pbWrapTarget.text = obj.name

on WrapMethod changed state do ( if WrapMethod.state == 4 then UnWrapMesh pbWrapTarget.object )			
	
on bGO pressed do 
	( 	SetWaitCursor()
		WrapSource = copy pbWrapSource.object 
		WrapTarget = copy pbWrapTarget.object ; ConvertToMesh WrapTarget

		format "WrapSource:% %\n" (superClassOf WrapSource) WrapSource.name
		format "WrapTarget:% %\n" (superClassOf WrapTarget) WrapTarget.name
		
		ShowNormals = cbShowNormals.state
		
		if WrapMethod.state == 4 then -- if Useing UVW Mapping
			(	if ConvertToSplineShape WrapSource == undefined then 
				( ConvertToMesh WrapSource; WrapMeshUVW() ) 
				else WrapSplineUVW()
			)
		else  -- if NOT Useing UVW Mapping
			(	if ConvertToSplineShape WrapSource == undefined then 
				( ConvertToMesh WrapSource; WrapMesh WrapMethod.state )
				else WrapSpline WrapMethod.state
			)
				
		delete WrapTarget
		SetArrowCursor()
	)	
)

WrapperFloater = newrolloutfloater "Wrapper" 142 265
addrollout WrapperRollout WrapperFloater


