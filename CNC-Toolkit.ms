--------------------------------------------------------------------------------
--  CNC Toolkit - Copyright 2009 Rab Gordon
--  email: rab@cnc-toolkit.com
--  www.cnc-toolkit.com
--  This MaxScript is distributed as free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
--  the Free Software Foundation, either version 3 of the License, or any later version.
--  This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
--	without even the implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
--  See the GNU General Public License for more details.
--  You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

--  Thanks and Acknowledgements:
--  For help and advice from swami*, a.k.a. "codeWarrior()
--  For the Roland File Format Import by Doug McNabb ~ this is where it all began.
--  For some functions and ideas inspired by John Burnett's MaxScript library

-- Change log 
-- 4.34 - "Digitise": -- Use Digitise Probe routine following toolpath
---------------------------------------------------------------------------------
---------------------------------------------------------------------------------

-- DEBUG = true -- just uncomment (remove the '--' at the start of this line) to enable DEBUG output to the MaxScript Listener

-- DECLARE FUNCTIONS, etc.
-- Function List
Global ZeroPointPos,Debug,InitialiseVariables,StartTimer1,EndTimer1,StartTimer2,EndTimer2,ImportZMatrixToPlane,SplineFilter,MeshFilter,padLeadingZeros,HAPFOLineNo,ResetXforms,dbpyr,difference,ImportCSVMatrix,NumberAsLimitedFloat,RoundOffNumber,HideVectors,UnHideVectors,readnumber,readCSVnumber,readtext,SceneSetup,CreateZeroPoint,CreateCutter,CreateAAxis,CreateBAxis,CreateCAxis,MakeTextRing,SplineFromObjects,create_preview_toolpath,ImportRolandToolpath,ImportGCODE,ImportPLC,ImportCSV,MakeDrillCycleToolPath,MakeParallelSplines,MakeToolpath,DigitisePeckToolpath,AppendToolpath,CalcToolAngleVector,GetClosestVertexVectorOLD,GetClosestVertexVectorNEW,GetClosestVertexVector,detach_splines,Is_Curved_Seg,Subdivide_Shape,SubdivideAndRenameShape,make_contour_lines,attach_splines,create_vertex_array,closest_vertex,check_vertices,fuse_vertices,weld_vertices,ProjectSpline,SplineFromMesh,PostProcessToolpath,GetMoveType,GetToolVector,CalcToolPathExportCoordsXYZ,CalcToolPathExportCoordsXYZA,CalcToolPathExportCoordsXYZBC,CalcToolPathExportCoordsXYZAC,CalcToolPathExportCoordsXYZAB,CalcToolPathExportCoordsRobotArm,CalcToolPathExportCoords,WriteToolpath,ToolpathExportString,CleanTPDialogs,LaunchCNCToolkitUI
-- Rollout list
Global CNCTKFloater,ToolpathGenerator_Rollout,TPFShapeDialog,TPFProjdShapeDialog,TPFStepoversDia,MachineTypeDialog,TPFDrillCycleDia
---------------------------------------------------------------------------------
fn InitialiseVariables = ( -- Initialise all Variables, but import values from ini file if available
	try (filein "CNC_Toolkit.ini" ; format "CNC_Toolkit.ini loaded\n") catch (format "CNC_Toolkit.ini not found\n")

	Global GMAX = (maxversion())[2] == 128 -- Check Max Version
		
	Global MachineTypes = #("XYZ","XZA","XYZA","XYZAB","XYZAC","XYZBC","Robot Arm")
	if MachineType == undefined then Global MachineType = 1
	
	Global ProjectionMethods = #("Downwards" , "Around X Axis" , "Around Y Axis" , "Around Z Axis")
	if ProjectionMethod == undefined then Global ProjectionMethod = 1
	
	Global StepoverMethods = #("Flat, Parallel to X-Axis","Flat, Parallel to Y-Axis","Flat, Spiral","Spiral around X-Axis","Spiral around Y-Axis","Spiral around Z-Axis" )
	if StepoverMethod == undefined then Global StepoverMethod = 1
	
	Global VectorCalcMethods = #("Don't Calculate Angles","Angle is Parallel to Z-axis","Angle from Origin","Angle Around X-Axis","Angle from Reference Spline","Angle from Reference Surface")	
	if VectorCalcMethod == undefined then Global VectorCalcMethod = 2
	
	Global ExportFormats = #("Standard G-Code","Digitise","Standard HPGL","Roland CNC HPGL","HAPFO-CNC","CSV","CamBam") 
	if ExportFormat == undefined then Global ExportFormat = 1
	
	Global ExportDestinations = #("File","Printer/Serial Port","Default File","Script Listener Window")
	if GMAX then Global ExportDestination = 4 
	if ExportDestination == undefined then Global ExportDestination = 1
	
	-- Waterline Splines variables
	if Dist_from_top == undefined then Global Dist_from_top= 1.0		
	if Dist_from_base == undefined then Global Dist_from_base= 1.0
	if Spacing == undefined then Global Spacing= 10.0
	if Fuse_n_Weld_threshold == undefined then Global Fuse_n_Weld_threshold= 1.0
	
	-- Parallel Splines variables
	if Stepover == undefined then Global Stepover = 10.0	
	
	if ProjectionPreSubdivide == undefined then Global ProjectionPreSubdivide = false	
	if ProjectionOffset == undefined then Global ProjectionOffset = 0.0	
	
	-- Spline Subdivision variables
	if pre_subdivide == undefined then Global pre_subdivide = true
	if min_Seg_length == undefined then Global min_Seg_length = 1.0
	if max_Seg_length == undefined then Global max_Seg_length = 5.0
	if Max_Curvature == undefined then Global Max_Curvature = 20.0
	if subdivide_recursions == undefined then Global subdivide_recursions = 5

	-- Toolpath from spline variables
	Global tool_clearance,VectorSize
	if tool_clearance == undefined then Global tool_clearance = 25.0
	Global VectorSize = (tool_clearance / 3)
	
	if ToolDiameter == undefined then Global ToolDiameter = 3.0
	if SpindleSpeed == undefined then Global SpindleSpeed = 1000.0
	
	if Dwell == undefined then Global Dwell = 0.25
	if DrillDepth == undefined then Global DrillDepth = 1.0	
	if PeckDepth == undefined then Global PeckDepth = 1.0	
	
	if CutDepth == undefined then Global CutDepth = 0.0
	if MultiPasses == undefined then Global MultiPasses = 1	
	if ToolOffset == undefined then Global ToolOffset = 0.0
	
	if MakeVectors == undefined then Global MakeVectors = false	
	
	-- Vector Calculations , default no. of checkbow
	if TangentMode == undefined then Global TangentMode = false	
	
	-- Toolpath Export & Options
	if Feedrate == undefined then Global Feedrate = 100.0
	if PlungeRatio == undefined then Global PlungeRatio = 50.0

	if OffsetAroundAaxis== undefined then Global OffsetAroundAaxis= 100.0
	if OffsetAroundBaxis== undefined then Global OffsetAroundBaxis= 100.0
	if OffsetAroundCaxis== undefined then Global OffsetAroundCaxis= 100.0
	
	if ClearanceMode == undefined then Global ClearanceMode= 1
		
	if ZDirection == undefined then Global ZDirection = 1
	if RecalcFeedrateEnabled == undefined then Global RecalcFeedrateEnabled = false

	if OutputPort == undefined then Global OutputPort = 1
	if DefaultFileName == undefined then Global DefaultFileName = sysInfo.currentdir + "\\tempfile.tap"

	if OutputLineNo == undefined then Global OutputLineNo = false -- Write Line Numbers	
	
	if SubRoutine == undefined then Global SubRoutine = false -- is GCode to be used as part of a subroutine ?
		
	if FixedZ == undefined then Global FixedZ = false 
	if FixedZDown == undefined then Global FixedZDown = "-1" 
	if FixedZUp == undefined then Global FixedZUp = "0" 
	
-------------------------------------------------------

Global Toolpath , Angle , Depth , AngleDiff , Revolutions , PathNormalAngle , clr , tmpstring , k1 , nearest_angle_zero , k , NearestAngleZero , ShapeToProcess , ToolAngleVector
Global start_at_Zero = true , end_at_Zero = true , preview_toolpath	= true
Global inFile , out
Global new_toolpath , ToolAngle , ReferenceSpline , ReferenceMesh , Cutter , Toolpivot , KnotDistances , ToolAngles , startTime1 , endTime1 , startTime2 , endTime2 , ToolpathHasVectors
Global ReferenceMesh , tempReferenceMesh 
Global MoveType  = "Rapid Move"

Global SixAxisCoords 			-- Data structure for 6 axes , ie. x ,y ,z coords plus angle of rotation around x ,y ,z planes
Struct SixAxisCoords (pos ,ang) 	-- pos. ~ x ,y ,z position -- ang.x ~ rotation around x-axis (4th-axis) , ang.y ~ rotation around y-axis (5th-axis) , ang.z ~ rotation around z-axis (6th-axis)
Global MachineCoords 			= SixAxisCoords [0.0 ,0.0 ,0.0] [0.0 ,0.0 ,0.0]
Global LastMachineCoords 	= SixAxisCoords [0.0 ,0.0 ,0.0] [0.0 ,0.0 ,0.0]
Global ClosestVertexCoords 	= SixAxisCoords [0.0 ,0.0 ,0.0] [0.0 ,0.0 ,0.0]
Global MachineCoordsStart	= SixAxisCoords [0.0 ,0.0 ,0.0] [0.0 ,0.0 ,0.0]


----------------------------------------------------------------------
	-- Setup Toolpath Material
	Global ToolpathMaterial
	ToolpathMaterial = Multimaterial numsubs:6
	ToolpathMaterial.name = "Toolpath" 
	ToolpathMaterial.materialList[1].Diffuse = color 255 0 0			;	ToolpathMaterial.materialList[1].name = "Feed Move"	
	ToolpathMaterial.materialList[2].Diffuse = color 23 121 0 		;	ToolpathMaterial.materialList[2].name = "Retract Move"	
	ToolpathMaterial.materialList[3].Diffuse = color 37 47 174 		;	ToolpathMaterial.materialList[3].name = "Rapid Move"
	ToolpathMaterial.materialList[4].Diffuse = color 150 255 209 	;	ToolpathMaterial.materialList[4].name = "Plunge Move"	
	ToolpathMaterial.materialList[5].Diffuse = color 255 200 0 		;	ToolpathMaterial.materialList[5].name = "Vector"
	ToolpathMaterial.materialList[6].Diffuse = color 155 0 155 		;	ToolpathMaterial.materialList[6].name = "Plunge/Dwell"
	
	setMeditMaterial 1 ToolpathMaterial 
	
	-- Setup Axis Material
	AxesMaterial = Multimaterial numsubs:5
	AxesMaterial.name = "Axes" 
	AxesMaterial.materialList[1].Diffuse = color 255 0 0		;	AxesMaterial.materialList[1].name = "X-Axis"		
	AxesMaterial.materialList[2].Diffuse = color 0 255 0 		;	AxesMaterial.materialList[2].name = "Y-Axis"	
	AxesMaterial.materialList[3].Diffuse = color 0 0 255 		;	AxesMaterial.materialList[3].name = "Z-Axis"
	AxesMaterial.materialList[4].Diffuse = color 60 60 60 		;	AxesMaterial.materialList[4].name = "Pointer"	
	AxesMaterial.materialList[5].Diffuse = color 255 255 255 ;	AxesMaterial.materialList[5].name = "Spare"
	setMeditMaterial 2 AxesMaterial
	
	AAxisMaterial = StandardMaterial name: "A-Axis" diffuse: (color 255 0 0) opacity: 15 ; setMeditMaterial 7 AAxisMaterial
	BAxisMaterial = StandardMaterial name: "B-Axis" diffuse: (color 0 255 0) opacity: 15  ; setMeditMaterial 8 BAxisMaterial
	CAxisMaterial = StandardMaterial name: "C-Axis" diffuse: (color 0 0 255) opacity: 15  ; setMeditMaterial 9 CAxisMaterial
	PointerMaterial = StandardMaterial name: "Pointer" diffuse: (color 60 60 60) opacity: 15  ; setMeditMaterial 4 PointerMaterial
	
)

-- MINOR FUNCTIONS--
fn HideVectors = (hideselectedsplines  $;	updateshape $)
fn UnHideVectors = (unhidesegments $;	updateshape $)
fn SplineFilter obj = SuperClassOf obj == Shape 
fn MeshFilter obj = SuperClassOf obj == GeometryClass
fn padLeadingZeros num numMaxChar=(n=numMaxChar-((numStr=num as string).count);for i=1 to n do numStr="0"+numStr;numStr)
fn HAPFOLineNo = (LineNo += 1 ; return "N" + (padLeadingZeros LineNo 6) + "  " as string )
fn dbpyr dbpyr_val size =( --Debugging tool
	pyramid width:size depth:size  height:size pos:dbpyr_val wirecolor: clr
)
fn difference val1 val2 = ( 	if val1 < val2 then diff1 = val2 - val1 else diff1 = val1 - val2
	if diff1 > 180 then diff1 = 180 - (diff1 - 180)
	return diff1	)
fn NumberAsLimitedFloat val =(
	val *= 1000; --print val
	val = val as integer; --print val
	val = val as float; --print val
	val /= 1000; --print val
	return val
)
fn RoundOffNumber n places:4 =(	
	local mult = pow 10 places
	n *= mult
	if n < 0 then n -= 0.5 else n += 0.5
	return ((n as integer) / mult)
)
fn readnumber file=(  
local str="" ; local Char ; local end_of_number = false
if readchar inFile != " " then seek inFile ( filepos inFile - 1 )		-- filter out first space if it exists
while not end_of_number do
	(	Char= readchar inFile;	--format "no: % \n" Char	
		if findString "-.1234567890E+" Char == undefined then end_of_number = true else Str += Char
	)
	number = execute Str
	seek inFile ( filepos inFile - 1 )
	return number
)
fn readCSVnumber file=(  
local str="[" ; local Char ; local end_of_number = false
if readchar inFile != " " then seek inFile ( filepos inFile - 1 ) -- filter out first space if it exists
while not end_of_number and not eof file do
	(	Char= readchar inFile ;	--if eof inFile do print "EOF" 
		if DEBUG then format "no: % \n" Char	
		if findString "-.1234567890,E" Char == undefined then end_of_number = true else Str += Char
	)
	str += "]"
	seek inFile ( filepos inFile - 1 )
	thePoint3 = execute str
	if DEBUG then format "thePoint3: %\n" thePoint3 
	return thePoint3 
)
fn readtext file=(  
local str="" ; local Char ; local number = false
	str = readchar inFile
	if findString "-.1234567890" str != undefined then return str

while ( not number and not eof inFile )do
	(	Char= readchar inFile;	--format "Char: % \n" Char
		if findString "-.1234567890" Char != undefined then number = true else Str += Char
	)
	return str
)
fn ResetXforms =(			
	for obj in selection do
	(	if DEBUG then Format "ResetXforms %\n" obj 
		old_ot = obj.objecttransform ; old_t = obj.transform ; offset = old_ot.row4 - old_t.row4 ; old_piv = obj.pivot
		obj.transform = matrix3 1 ;	obj.objectoffsetpos = [0,0,0] ;	obj.objectoffsetrot = quat 1 ; obj.objectoffsetscale = [1,1,1]
		mody = xform() ; addmodifier obj mody ;	mody.gizmo.transform = old_ot ;	mody.gizmo.position = offset ; obj.position = old_piv
		Local extents=[0,0,0] ; extents.x=abs (obj.max.x-obj.min.x) ; extents.y=abs (obj.max.y-obj.min.y) ; extents.z=abs (obj.max.z-obj.min.z)
		obj.pivot = [obj.min.x,obj.min.y,obj.min.z] + extents / 2
	collapseStack obj
	)	
)	

-- TIMER1 --
fn StartTimer1 = (format "Starting Timer\n" ; startTime1 = TimeStamp())
fn EndTimer1 timingof1 =(	
	endTime1 = TimeStamp()
	local midNiteAdj, dt, dtMin, dtSec
	midNiteAdj=if startTime1<endTime1 then 0 else 86400000	-- Handle midnite spanning.
	dt=(endTime1-startTime1+midNiteAdj)/1000.

	if dt>=60 then
	(	-- We are here if dt is one minute or more.
		dtMin=(dt/60.) as integer
		dtSec=(mod dt 60) as integer
	)	else	( dtMin=0 ; dtSec=dt ) -- Less than a Minute
	
	-- RoundOffNumber off the seconds.
	dtSec*=10^2 ; dtSec= (dtSec+0.5) as integer ; dtSec/=((10^2) as float)
	format "% Timed at; % Min, % sec \n" timingof1 dtMin dtSec 
)

-- TIMER2 -- For timing within another timed process
fn StartTimer2 = (format "Starting Timer2\n" ; startTime2 = TimeStamp())
fn EndTimer2 timingof2 =(	
	endTime2 = TimeStamp()
	local midNiteAdj, dt, dtMin, dtSec
	midNiteAdj=if startTime2<endTime2 then 0 else 86400000	-- Handle midnite spanning.
	dt=(endTime2-startTime2+midNiteAdj)/1000.

	if dt>=60 then
	(	-- We are here if dt is one minute or more.
		dtMin=(dt/60.) as integer
		dtSec=(mod dt 60) as integer
	)	else	( dtMin=0 ; dtSec=dt ) -- Less than a Minute
	
	-- RoundOffNumber off the seconds.
	dtSec*=10^2 ; dtSec= (dtSec+0.5) as integer ; dtSec/=((10^2) as float)
	format "% Timed at; % Min, % sec \n" timingof2 dtMin dtSec 
)

-- SCENE SETUP (MACHINE CONFIGURATION) --
function SceneSetup mode:"export"= ( 
if DEBUG then format "fn SceneSetup: %\n mode: %\n" MachineTypes[MachineType] mode
	Global AAxis, BAxis, CAxis, ZeroPoint, Cutter
	
	try (delete AAxis) catch() ;try (delete BAxis) catch() ; try (delete CAxis) catch()
	try (delete $AAxis) catch() ;try (delete $BAxis) catch() ; try (delete $CAxis) catch()

	CreateZeroPoint() ; CreateCutter()

	case MachineTypes[MachineType] of
		(
		"XYZ"		:()
		"XYZA"	:(CreateAAxis())
			
		"XZA"		:(CreateAAxis())	
			
		"XYZAB"	:(
					CreateAAxis();CreateBAxis()
					BAxis.pos =[0,0,(ZeroPoint.pos.z + OffsetAroundBAxis)]
					Cutter.pos = ZeroPoint.pos
			
						if mode == "export" then
						(	-- Links For Export
						BAxis.parent = Cutter
						)
						else
						(	-- Links for Import
						Cutter.parent= BAxis
						)		
					)
			
		"XYZBC"	:(CreateBAxis();CreateCAxis())
		
		"XYZAC"	:(
					CreateAAxis();CreateCAxis()
					
					AAxis.ObjectOffsetPos = [(-OffsetAroundCAxis + (OffsetAroundAAxis/200)),0,0]
					AAxis.pos = ZeroPoint.pos + [0,0,OffsetAroundAAxis]
						
					CAxis.ObjectOffsetRot = (quat 0 0 0 1)

					CAxis.ObjectOffsetPos = [0,0,0]
					CAxis.pos = ZeroPoint.pos + [-OffsetAroundCAxis,0,OffsetAroundAAxis]
					Cutter.pos = ZeroPoint.pos
						
					if mode == "export" then
						(	-- Links For Export
						AAxis.parent = Cutter
						CAxis.parent = AAxis; setInheritanceFlags CAxis #{1..3, 6..9}
						)
						else
						(	-- Links for Import
						Cutter.parent= AAxis
						AAxis.parent = CAxis
						)
					)
		)
if DEBUG then format "Scene Now Setup\n"
)
function CreateZeroPoint =(	
	if DEBUG then format "fn CreateZeroPoint\n"
	if IsValidNode $ZeroPoint then ZeroPointPos = $ZeroPoint.pos else ZeroPointPos = [0,0,0]
	try (delete $ZeroPoint ) Catch()
	ZeroPoint = Point pos:ZeroPointPos isSelected:off Box:off axistripod:on wirecolor:red centermarker:on size:50 constantscreensize:on drawontop:on name:"ZeroPoint"
)
function CreateCutter =(	
	if DEBUG then format "fn CreateCutter\n"
	try (delete $Cutter) Catch() 
	if GMAX then Cutter = Cylinder fillet:1 wirecolor:black opacity:20 smooth:on heightsegs:1 capsegs:1 sides:6 height:OffsetAroundAaxis radius:(ToolDiameter/2) name:"Cutter"
	else Cutter = ChamferCyl fillet:1 wirecolor:black opacity:20 smooth:on heightsegs:1 capsegs:1 sides:6 height:OffsetAroundAaxis radius:(ToolDiameter/2) name:"Cutter"
	Cutter.dir = [0,0,1]
	in coordsys local Cutter.rotation = quat -90 z_axis
)	
function CreateAAxis =(	
	if DEBUG then format "fn CreateAAxis\n"
	Local AxisSize = OffsetAroundAaxis
	try (delete AAxis) Catch()
	if IsValidNode AAxis then Position = AAxis.pos 
	AAxis = Tube smooth:on sides:22 wirecolor:red capsegs:1 heightsegs:1 radius1:AxisSize radius2:(AxisSize/1.1) height:(AxisSize/10) name:"A-Axis" material:AAxisMaterial
	AAxisPointer = Cylinder smooth:off wirecolor:gray heightsegs:1 capsegs:1 sides:3 height:(AxisSize/9) radius:(AxisSize/9) material:PointerMaterial 
	AAxis.pos.x = AAxis.pos.x + (AxisSize/20)
	AAxisPointer.pos.x = AAxis.pos.x + (AxisSize/1.05)
	AAxisPointer.pos.z = AAxis.pos.z - (AxisSize/200)
	convertToMesh AAxis 
	attach AAxis AAxisPointer
	--AAxis.ObjectOffsetRot = quat 90 y_axis 
	AAxis.ObjectOffsetRot = quat -90 y_axis
)
function CreateBAxis =(	
	if DEBUG then format "fn CreateBAxis\n"
	Local AxisSize = OffsetAroundBAxis
	try (delete BAxis) Catch()
	if IsValidNode BAxis then Position = BAxis.pos 
	BAxis = Tube smooth:on sides:22 wirecolor:green capsegs:1 heightsegs:1 radius1:AxisSize radius2:(AxisSize/1.1) height:(AxisSize/10) name:"B-Axis" material:BAxisMaterial
	BAxisPointer = Cylinder smooth:off wirecolor:gray heightsegs:1 capsegs:1 sides:3 height:(AxisSize/9) radius:(AxisSize/9) material:PointerMaterial 
	BAxisPointer.pos.x = BAxis.pos.x + (AxisSize/1.05)
	BAxisPointer.pos.z = BAxis.pos.z - (AxisSize/200)
	convertToMesh BAxis 
	attach BAxis BAxisPointer
	BAxis.ObjectOffsetRot = (quat 1 1 -1 -1)
)
function CreateCAxis =(	
	if DEBUG then format "fn CreateCAxis\n"
	Local AxisSize = OffsetAroundCAxis
	try (delete CAxis) Catch()
	if IsValidNode CAxis then Position = CAxis.pos 
	CAxis = Tube smooth:on sides:22 wirecolor:blue capsegs:1 heightsegs:1 radius1:AxisSize radius2:(AxisSize/1.1) height:(AxisSize/10) name:"C-Axis" material:CAxisMaterial
	CAxisPointer = Cylinder smooth:off wirecolor:gray heightsegs:1 capsegs:1 sides:3 height:(AxisSize/9) radius:(AxisSize/9) material:PointerMaterial 
	CAxisPointer.pos.x = CAxis.pos.x + (AxisSize/1.05)
	CAxisPointer.pos.z = CAxis.pos.z - (AxisSize/200) 
	convertToMesh CAxis 
	attach CAxis CAxisPointer
	CAxis.ObjectOffsetRot = (quat 0 0 1 1)
)

-- UTILITIES --
function MakeTextRing =(
text size:100 pos:[0,0,0] isSelected:on text:"CNC Toolkit"
rotate $ (angleaxis 90 [0,0,1])
modPanel.addModToSelection (Bend ()) ui:on
$.modifiers[#Bend].BendAngle = 360
$.modifiers[#Bend].BendAxis = 0
Local extents=[0,0,0] ; extents.x=abs ($.max.x-$.min.x) ; extents.y=abs ($.max.y-$.min.y) ; extents.z=abs ($.max.z-$.min.z)
MinPoint = [$.min.x,$.min.y,$.min.z] ; MaxPoint = [$.max.x,$.max.y,$.max.z]
center = MinPoint = [$.min.x,$.min.y,$.min.z] + extents / 2
$.pivot = center ; $.pos = [0,0,0] ; rotate $ (angleaxis -180 [1,0,0])
max tool zoomextents all 
)
function SplineFromObjects =( 	-- Create a Spline from a Selection of Objects, useful for getting a toolpath from digitised points
	NewShape = splineShape prefix: "Spline From Objects"
    TmpSpline = addNewSpline NewShape
	for obj in Selection do
		(
		pos = obj.pos
		format "% \t %\n" obj.name pos
		addKnot NewShape TmpSpline #corner #line pos
		)
	try (updateshape NewShape) catch()
    select NewShape 	
)
function create_preview_toolpath =  (
    p=$
	t = Point pos:[0,0,0] isSelected:off Box:off axistripod:on wirecolor:Blue centermarker:on size:tool_clearance constantscreensize:off drawontop:on name:"PreviewGizmo"
   	t.pos.controller=path follow:false
   	tool_control=t.pos.controller
   	tool_control.path=p
	tool_control.constantVel = true
	nk = numknots p 1
	animationrange = (interval 1 nk)
   	animate on ( at time 0 tool_control.percent=0 at time 1000 tool_control.percent=100 )
	sliderTime = 0f
	playAnimation()
  )

-- IMPORT --
function ImportRolandToolpath =  ( -- Original script for importing the Roland File Format written by Doug McNabb
    inFileName = getOpenFileName caption:"Select Roland ASCII File" types:"Roland Files (*.rol)|*.ROL|Text files (*.txt)|*.TXT|All Files (*.*)|*.*|"
    ( -- open specified file
      inFile = openFile inFileName mode:"r"
      if inFile == undefined then messagebox "There was an error opening the File!" title:"Roland file import" else
      ( -- Create the splineShape and add the spline
        Imported_Spline = splineShape prefix: "Imported_Spline"
        tmpSpline = addNewSpline Imported_Spline

        while not eof inFile do        --loop through the file until the end again...
        ( tmpChar = readchar inFile
          case tmpChar of
          ( "Z" : 	( -- "Z" starts the vertex data. -- read file until we have X, Y, and Z coordinates.
              		coords = "["
              		tmpChar = readchar inFile -- skip the "Z"
              		while  tmpChar != ";" and tmpChar != "\n" and not eof inFile do
              			( -- TODO: add check. tmpChar must be either a number, a comma, or a space.
                		coords  = coords + tmpChar
                		tmpChar = readchar inFile
              			)
		             coords = coords + "]"
		             thePoint3 = execute coords
		             addKnot Imported_Spline tmpSpline #corner #line thePoint3
            		) -- "Z"
          ) -- case
        ) -- while not eof
       	close inFile
        updateShape Imported_Spline ; select Imported_Spline
      ) -- file open
    ) -- get filename
  ) -- import_toolpath
function ImportGCODE =  (   
  	if not IsValidNode ZeroPoint then (messageBox "You must have a valid Zero Point, re-run CNC Toolkit to create one." ; return 0)
	inFileName = getOpenFileName caption:"Select G_Code File" types:"All Files (*.*)|*.*|TAP(*.tap)|*.tap|Text files (*.txt)|*.TXT|"

    ( inFile = openFile inFileName mode:"r"
      if inFile == undefined then messagebox "There was an error opening the File!" title:"G_Code file import" else
      (
        Imported_Spline = splineShape prefix: inFileName
        tmpSpline = addNewSpline Imported_Spline
        Local coords = [0.0,0.0,0.0]
		Local Movemade = false

        while not eof inFile do
        (	
			tmpChar = readchar inFile
			--format " FilePos % \n" (filepos inFile);		format ": % \n" tmpChar

          case tmpChar of
          (
            "X" : 	( coords.x = readnumber inFile; movemade=true)

            "Y" : 		( coords.y = readnumber inFile; movemade=true)

            "Z" : 	( coords.z = readnumber inFile; movemade=true)

			"A" : 	(	Local ToolpathAngleA =readnumber inFile 
						--	format "Rotating % \n" angle --	try (updateShape Imported_Spline) catch()
						Imported_Spline.rotation = quat ToolpathAngleA x_axis
						movemade=true
						)
				
					/*					
			"B" : 	(	Local ToolpathAngleB =readnumber inFile 
					-- 	format "Rotating % \n" angle --	try (updateShape Imported_Spline) catch()
					Imported_Spline.rotation = quat ToolpathAngleB x_axis
					movemade=true
					)	

			"C" : 	(	Local ToolpathAngleC =readnumber inFile 
					--	format "Rotating % \n" angle --	try (updateShape Imported_Spline) catch()
					Imported_Spline.rotation = quat ToolpathAngleC x_axis
					movemade=true
					)	
					*/

			"\n" : ( --format " End of Line \n"
                	if movemade==true then
						( 
						addKnot Imported_Spline tmpSpline #smooth #line (coords + $ZeroPoint.pos)
						--format "Knot % \n \n" (numKnots Imported_Spline)
						--ToolAngle = addNewSpline Imported_Spline
						--addKnot new_toolpath ToolAngle #corner #line coords 
						--addKnot new_toolpath ToolAngle #corner #line (coords *2)
						--if (numKnots Imported_Spline) > 1 then updateShape Imported_Spline
						)
					Movemade = false
					)

			"("	:	( format " Comment \n" ; skipToNextLine inFile )
			
			"*"	:	( format " Comment \n" ; skipToNextLine inFile )

			"G" :	( Local GCommand=readnumber inFile ; format "G% \n" GCommand )

			"F" :	( Local Feedrate=readnumber inFile ; format "F% \n" Feedrate )

			--"N":	( --	format "Line Number"
				--	LineNumber = readvalue inFile --	seek inFile ( filepos inFile - 1 )
				--	)

          ) -- case

		)

        ) -- while not eof
		)
        close inFile
		try (updateShape Imported_Spline)  catch()
		select Imported_Spline
  )
function ImportPLC =  (   
  	if not IsValidNode ZeroPoint then (messageBox "You must have a valid Zero Point, re-run CNC Toolkit to create one." ; return 0)
	inFileName = getOpenFileName caption:"Select G_Code File" types:"Text files (*.txt)|*.TXT|PLC(*.plc)|*.plc|All Files (*.*)|*.*|"

    ( inFile = openFile inFileName mode:"r"
      if inFile == undefined then messagebox "There was an error opening the File!" title:"PLC file import" else
      (
        Imported_Spline = splineShape prefix: inFileName ; tmpSpline = addNewSpline Imported_Spline
		Imported_Spline.baseobject.renderable=true ; Imported_Spline.material = ToolpathMaterial
        Local coords = [0.0,0.0,0.0] ; Local MoveType  = 1

	        while not eof inFile do
	        (--format " FilePos % \n" (filepos inFile)
	       
			tmpChar = readtext inFile ;	--format "tmpChar: % \n" tmpChar
			
			if (findstring tmpChar "Rapids") != undefined then
				(	format "Rapid \n"
					MoveType  = 3
				)
			
			if (findstring tmpChar "Toolpaths") != undefined then 
				(	format "Toolpath \n"
				  	MoveType  = 1
				)
			
			if (findstring tmpChar "Line") != undefined then 
				(format "Line \n"
				seek inFile ( filepos inFile - 1 )
				coords.x = readnumber inFile ; readchar inFile
				coords.y = readnumber inFile ; readchar inFile
				coords.z = readnumber inFile 
				--format "coords %\n" coords 
				
				addKnot Imported_Spline tmpSpline #corner #line (coords + $ZeroPoint.pos)
				
				if (numKnots Imported_Spline) > 1 then (setMaterialID Imported_Spline 1 (numsegments Imported_Spline 1) MoveType  ; updateShape Imported_Spline )
				)

			)
       	) 
	)
        close inFile
        updateShape Imported_Spline; select Imported_Spline
  )
function ImportCSV =  ( 
	  inFileName = getOpenFileName caption:"Select Comma Delimited File" types:"CSV (*.csv)|*.CSV|Text files (*.txt)|*.TXT|All Files (*.*)|*.*|"
    ( -- open specified file
      inFile = openFile inFileName mode:"r"
	  SplineName = inFileName as String
	  format "Importing %\n" inFileName 
      if inFile == undefined then messagebox "There was an error opening the File!" title:"CSV file import" else
      ( -- Create the splineShape and add the spline
        Imported_Spline = splineShape name: SplineName 
        tmpSpline = addNewSpline Imported_Spline
        while not eof inFile do        --loop through the file until the end again...
        (           
					tmpChar =  ""		
					Character = true
					tmpChar = readchar inFile
					
					while Character == true and not eof inFile do
					(					
					if findString "-.1234567890" tmpChar == undefined then 
					
						(	Character = true ; tmpChar = readchar inFile )
					else
						( 	
							Character = false
						)
					)
					
					
					if not eof inFile do
					 (
					seek inFile ( filepos inFile - 1 )
					
					coords = readCSVnumber inFile
					coords += $ZeroPoint.pos
					addKnot Imported_Spline tmpSpline #corner #line coords
					--addKnot Imported_Spline tmpSpline #smooth #curve coords
					)

					
        ) -- while not eof
       	close inFile
        updateShape Imported_Spline ; select Imported_Spline
      ) -- file open
    ) -- get filename
  ) 
function ImportCSVMatrix = ( 
	  inFileName = getOpenFileName caption:"Select Comma Delimited File" types:"CSV (*.csv)|*.CSV|Text files (*.txt)|*.TXT|All Files (*.*)|*.*|"
    ( -- open specified file
      inFile = openFile inFileName mode:"r"
	  SplineName = inFileName as String
	  format "Importing %\n" inFileName 
      if inFile == undefined then messagebox "There was an error opening the File!" title:"CSV file import" else
      ( -- Create the splineShape and add the spline
        Imported_Spline = splineShape name: SplineName 
     --   tmpSpline = addNewSpline Imported_Spline
		 Local coords = [0.0,0.0,0.0] 
		 Local Char = ""
		 Local IsANumber = false
		 Local EndOfLine = true
		 Local LineGoingRight = true
		 Local Cols = 0
		 Local Rows = 0
        
		  while not eof inFile do        --loop through the file until the end again...
			(           
					Local Char =""
					Local IsANumber = false
							
					Char= readchar inFile
					format "Char % \n" Char
					--seek inFile ( filepos inFile - 1 )
					
					if Char == "\n" then 
						(format "EndOfLine %\n" EndOfLine 
							EndOfLine = true ; IsANumber = false
							coords.y +=1 ; Rows += 1
							if LineGoingRight == true then LineGoingRight = false else LineGoingRight = true 
							
						)	else
						(
						seek inFile ( filepos inFile - 1 )
						Local Number = readvalue inFile
						IsANumber = true	
						format "Number: %\n" Number
						)
						
					if IsANumber == true then 
					(
					if EndOfLine then (
											tmpSpline = addNewSpline Imported_Spline
											try ( updateShape Imported_Spline ) catch ()
											)
					--Local Number = execute (readDelimitedString inFile ",")
					
					coords.z = Number
					coords += $ZeroPoint.pos
						 
					addKnot Imported_Spline tmpSpline #corner #line coords
					-- try ( updateShape Imported_Spline ) catch ()
					-- addKnot Imported_Spline tmpSpline #smooth #curve coords
					if LineGoingRight == true then coords.x += 1 else coords.x -= 1 
					Cols += 1
					) 
					
				Local EndOfLine = false	
					
			)
       	close inFile
       try ( updateShape Imported_Spline ) catch ()
			select Imported_Spline
			Cols = Cols / Rows
			format "Cols:% Rows% \n" Cols Rows
      ) -- file open
    ) -- get filename
  ) 
function ImportZMatrixToPlane = (  
	GC()
	  inFileName = getOpenFileName caption:"Select Comma Delimited File" types:"CSV (*.csv)|*.CSV|Text files (*.txt)|*.TXT|All Files (*.*)|*.*|"
    ( -- open specified file
      inFile = openFile inFileName mode:"r"
	  SplineName = inFileName as String
	  format "Importing %\n" inFileName 
      if inFile == undefined then messagebox "There was an error opening the File!" title:"CSV file import" else
      ( -- Create the splineShape and add the spline
     --   Imported_Spline = splineShape name: SplineName 
      --  tmpSpline = addNewSpline Imported_Spline
		Local coords = [0.0,0.0,0.0] 
		Local LineGoingRight = true
		Local Cols = 0
		Local Rows = 0
		Local EndOfLine = true
		Local Numbers = 0
		  
		  while not eof inFile do        --loop through the file until the end again...
			(       
							if EndOfLine then 
							(
						--	tmpSpline = addNewSpline Imported_Spline
						--	try ( updateShape Imported_Spline ) catch ()
							) Local EndOfLine = false
							
					Local IsANumber = false
					Local Char = readchar inFile
					if findString "-.1234567890E+" Char != undefined then IsANumber = true ; --format "IsANumber %\n" IsANumber
						
					if Char == "\n" then 
						(--format "EndOfLine %\n" EndOfLine 
							EndOfLine = true 
							coords.y +=1 
							Rows += 1
							coords.x = 0 
							if LineGoingRight == true then LineGoingRight = false else LineGoingRight = true 

						)
						
					if IsANumber == true then 
						(
						Numbers += 1	
						seek inFile ( filepos inFile - 1 )
						--Local Number = execute (readDelimitedString inFile ",")
						Local Number = readnumber inFile
						coords.z = Number
						coords += $ZeroPoint.pos
					--	addKnot Imported_Spline tmpSpline #corner #line coords
						--addKnot Imported_Spline tmpSpline #smooth #curve coords
						--if LineGoingRight == true then coords.x += 1 else coords.x -= 1 
						coords.x += 1
						Cols += 1
						) 
			)
			
			try ( updateShape Imported_Spline ) catch ()
			--select Imported_Spline
			
			
			Cols = Cols / Rows
			format "Cols:% Rows:% Rows*Cols:% Numbers:% \n" Cols Rows (Rows * Cols) Numbers
						
			seek inFile 0
			
			ThePlane = Plane realWorldMapSize:off length:(Rows - 1) width:(Cols - 1) lengthsegs:(Rows - 1)  widthsegs:(Cols - 1)  pos:[((Cols - 1) /2.0),((Rows - 1)/2.0),0]
			ConvertToMesh ThePlane 
			
			Local n = 1
			
			 while not eof inFile do        --loop through the file until the end again...
			(           
						coords = getVert ThePlane n
						coords.z = readvalue infile
						coords += $ZeroPoint.pos	
							
						SetVert ThePlane n coords	
						n += 1
						 
			)
			close inFile
			format "n: %\n" n
			
      ) -- file open
    ) -- get filename
  ) 

-- CREATE TOOLPATHS --
function MakeDrillCycleToolPath =(
Local PeckCycles = DrillDepth / PeckDepth as integer
Global new_toolpath = splineShape prefix: ("Toolpath_" + ($selection[1].name as string))
Global ToolPosition = addNewSpline new_toolpath
new_toolpath.baseobject.renderable=true ; new_toolpath.material = ToolpathMaterial

AppendToolpath ZeroPoint.pos "Start/End"	

	for obj in Selection do
		(
		Local pos = obj.pos
		Local dir = obj.dir
		format "\npos: % \t dir: %\n" pos dir
		AppendToolpath pos  "Position+Clearance"	
		setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 3 -- Rapid
	
		for i = 1 to PeckCycles do
			(
			pos.z = -i * PeckDepth 
			
			AppendToolpath pos "Plunge/Dwell"
			setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 6 -- Plunge
			
			AppendToolpath obj.pos "Position+Clearance"	
			setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 2 -- Retract
			)
				if pos.z != DrillDepth then 
				(				
				pos.z = - DrillDepth
				
				AppendToolpath pos "Plunge/Dwell"
				setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 6 -- Plunge
				
				AppendToolpath obj.pos "Position+Clearance"	
				setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 2 -- Retract		
				)
		)

	AppendToolpath ZeroPoint.pos "Start/End"	
	setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 3 -- Rapid
	
	updateshape new_toolpath; select new_toolpath
)
function MakeParallelSplines mode Stepover x y r =(		
		Global pos = [0,0,0]
	 	new_toolpath = splineShape prefix: "Parallel_Lines"
    	tmpSpline = addNewSpline new_toolpath

case mode of 
	(
	1:		-- X Horizontal
			(	ysteps = (y / (2* Stepover)) + 1
				xsteps = (x / (Stepover)) + .5
					for ystep = 1 to ysteps do
					(	
					if pos.y <= y then
						( 
						addKnot new_toolpath tmpSpline #corner #line pos
	 					for xstep = 1 to xsteps do addKnot new_toolpath tmpSpline #corner #line (pos += [stepover,0,0])
						)
					if pos.y + Stepover <= y then
						( 
						addKnot new_toolpath tmpSpline #corner #line (pos += [0,stepover,0])
						for xstep = 1 to xsteps do addKnot new_toolpath tmpSpline #corner #line (pos -= [stepover,0,0])
						)
					pos += [0,stepover,0]
					)
			)
				
	2:		-- Y Horizontal
			(	xsteps = (x / (2* Stepover)) + 1
				ysteps = (y / (Stepover)) + .5
					for xstep = 1 to xsteps do
					(	
					if pos.x <= x then
						( 
						addKnot new_toolpath tmpSpline #corner #line pos
	 					for ystep = 1 to ysteps do addKnot new_toolpath tmpSpline #corner #line (pos += [0,stepover,0])
						)
					if pos.x + Stepover <= x then
						( 
						addKnot new_toolpath tmpSpline #corner #line (pos += [stepover,0,0])
						for ystep = 1 to ysteps do addKnot new_toolpath tmpSpline #corner #line (pos -= [0,stepover,0])
						)
					pos += [stepover,0,0]
					)
			)

		
	3: 		-- Spiral
			(	new_toolpath = Helix radius1:0 radius2:r height:0 turns:(r/Stepover) bias:0 direction:0 pos:[0,0,0] isSelected:on	
			)
				
	4: 		-- Radial (Rotary)
			( 	MakeParallelSplines 1 Stepover x (2* pi * r) r
						
					modPanel.addModToSelection (Bend ()) ui:on
					$.modifiers[#Bend].BendAngle = 360
					$.modifiers[#Bend].BendAxis = 0
					$.modifiers[#Bend].BendDir = 90
					$.modifiers[#Bend].BendAxis = 1
					Converttosplineshape $
					local extents=[0,0,0]
					--extents.x=abs ($.max.x-$.min.x)
					extents.y=abs ($.max.y-$.min.y)
					extents.z=abs ($.max.z-$.min.z)
					MinPoint = [$.min.x,$.min.y,$.min.z]
					MaxPoint = [$.max.x,$.max.y,$.max.z]
					center = [$.min.x,$.min.y,$.min.z] + extents / 2
					$.pivot = center 
					$.pos = [0,0,0]
					$.dir = [0,0,-1]
			)
				
	5: 		-- Helix (Rotary)
			(	new_toolpath = Helix radius1:r radius2:r height:x turns:(x/Stepover) bias:0 direction:0 pos:[0,0,0] isSelected:on
				new_toolpath.dir = [1,0,0]
			)			
	)
	try (updateshape new_toolpath) catch()
    select new_toolpath  
)
function MakeToolpath =(
	if $ == undefined then (messageBox "Select Splines / Shapes to Convert to a Toolpath" ; return 0)
	if not IsValidNode ZeroPoint then (messageBox "You must have a valid Zero Point, re-run CNC Toolkit to create one." ; return 0)
	if DEBUG then format "MakeToolpath\nVectorCalcMethod: %\n" VectorCalcMethod	
   	
	Global new_toolpath = splineShape prefix: ("Toolpath:" + ($selection[1].name as string))
	new_toolpath.baseobject.renderable=true ;	new_toolpath.material = ToolpathMaterial ; new_toolpath.DisplayRenderMesh=false
	Global ToolPosition = addNewSpline new_toolpath
	Global ToolAngles = #(), ThisKnifeAngle, LastKnifeAngle = [0,0,0] , LastPoint , LastOffsetPoint = [0,0,0]
	
	TargetDepth = CutDepth
		
	if VectorCalcMethods[VectorCalcMethod] == "Angle from Reference Surface" or VectorCalcMethods[VectorCalcMethod] == "Tangent to Surface" then 
										   ( if not IsValidNode ReferenceMesh then ( messagebox "Pick Reference Surface from Vector Calculations Menu" ; return 0)
											tempReferenceMesh = copy ReferenceMesh ; ConvertToMesh tempReferenceMesh 
											Global VertexArray= #() ; VertexArray[tempReferenceMesh.numVerts] = undefined
											VertexArray = for i = 1 to VertexArray.count collect (GetVert tempReferenceMesh i)
											)
									
	AppendToolpath ZeroPoint.pos "Start/End"	
	if MakeVectors then setMaterialID new_toolpath ToolAngle 1 5 -- Vector	

 for ShapeInSelection in Selection as Array do  						-- Number of shapes loop
 ( 			
	for Pass = 1.0 to MultiPasses do
	( CutDepth = (Pass / MultiPasses) * TargetDepth ;	if DEBUG then format "CutDepth: %\n" CutDepth
	
		if DEBUG then Format "Processing % \n" ShapeInSelection 
		if SuperClassOf ShapeInSelection == SHAPE then
		(	ShapeToProcess = Copy ShapeInSelection
			Convertto ShapeToProcess splineshape
			ResetXforms
			if DEBUG then format "ShapeToProcess: %\n" ShapeToProcess
			if pre_subdivide then Subdivide_Shape ShapeToProcess
		
			  for pns = 1 to numsplines ShapeToProcess do	-- Number of splines loop
		      (
				nk = numknots ShapeToProcess pns
		        for pnk = 1 to nk do				-- Number of knots loop
			        (k = getknotpoint ShapeToProcess pns pnk
					
						   if ToolOffset != 0 then			-- Use for Drag Knife Cutting
						   (--format "\nKnot: %\n" pnk
							 if pnk == 1 then ThisKnifeAngle = normalize (k-(getknotpoint ShapeToProcess pns (pnk + 1))) else ThisKnifeAngle = normalize (k - LastPoint) -- format "ThisKnifeAngle: %\n" ThisKnifeAngle
							 KnifeAngleDelta = dot ThisKnifeAngle LastKnifeAngle -- format "KnifeAngleDelta: % \n" KnifeAngleDelta
							 KnifeAngleDelta = 180 - (abs (acos KnifeAngleDelta)) -- format "KnifeAngleDelta: % \n" KnifeAngleDelta	
		
							 if abs KnifeAngleDelta < 0 then 
								 (--format "Big Angle\n"
								 kk = (LastOffsetPoint + ( ToolOffset * (normalize(rotatez (MatrixFromNormal ThisKnifeAngle) -135).row3)))
								 AppendToolpath kk "Position" ;	 LastOffsetPoint = kk
								 kk = (LastOffsetPoint + ( ToolOffset * (normalize(rotatez (MatrixFromNormal ThisKnifeAngle) -225).row3)))
								 AppendToolpath kk "Position" ;	 LastOffsetPoint = kk							 
								 kk = (LastOffsetPoint + ( ToolOffset * (normalize(rotatez (MatrixFromNormal ThisKnifeAngle) -315).row3)))
								 AppendToolpath kk "Position" ;	 LastOffsetPoint = kk					
								-- kk = (LastOffsetPoint + ( ToolOffset * (normalize(rotatez (MatrixFromNormal ThisKnifeAngle) -405).row3)))
								-- AppendToolpath kk "Position" ;	 LastOffsetPoint = kk
								)
							 LastPoint = k ; k = k + ( ToolOffset * ThisKnifeAngle )						 
							)
		
			        if pnk == 1 then					--Plunge		-- if first knot then first raise tool to tool_clearance
				        (--format "Toolangle %\n" toolangle
						AppendToolpath k "Position+Clearance"
						if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
						setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 3 -- Rapid
						AppendToolpath k "Plunge"
						if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
						setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 4 -- Plunge
			    		) 
						else -- for all other knots between first and last
						(	
						AppendToolpath k "Position"				--Standard knot
						if MakeVectors then setMaterialID new_toolpath ToolAngle 1 5 -- Vector
						setMaterialID new_toolpath 1 (numsegments new_toolpath 1) 1 -- Feedmove
						)
							   
					if pnk == nk then					--Retract -- If last knot -- 
						( 
						if (isclosed ShapeToProcess pns) then pnk = 1	-- If shape is closed then go back to first knot to finish shape
			           	k = getknotpoint ShapeToProcess pns pnk
							if ToolOffset != 0 then
							(
							 ThisKnifeAngle = normalize (k - LastPoint)
							 LastPoint = k
							 k = k + ( ToolOffset * ThisKnifeAngle )
						 	)
						AppendToolpath k "Position"
						if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
						setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 1 -- Feedmove
						AppendToolpath k "Position+Clearance"						
						if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
						setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 2 -- Retract
					   	)
						 
						 LastKnifeAngle = ThisKnifeAngle 
						 LastOffsetPoint = k
				     )
			   )
			delete ShapeToProcess
		  )
	) --Multipass
	) --ShapeInSelection
	
	AppendToolpath ZeroPoint.pos "Start/End"		
	try (new_toolpath.wirecolor = red) catch() 	
	if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
	setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 3 -- Rapid	
	TargetDepth = CutDepth
	-- Finish off Toolpath
	-- AppendToolpath k 3
	if VectorCalcMethods[VectorCalcMethod] == "Angle from Reference Surface" or VectorCalcMethods[VectorCalcMethod] == "Tangent to Surface" then delete tempReferenceMesh	
	updateshape new_toolpath; select new_toolpath
	--setSplineSelection new_toolpath ToolAngles
  )
  
  function DigitisePeckToolpath =(
	if $ == undefined then (messageBox "Select Splines / Shapes to Convert to a Toolpath" ; return 0)
	if not IsValidNode ZeroPoint then (messageBox "You must have a valid Zero Point, re-run CNC Toolkit to create one." ; return 0)
	if DEBUG then format "DigitisePeckToolpath\nVectorCalcMethod: %\n" VectorCalcMethod	
   	
	Global new_toolpath = splineShape prefix: ("Toolpath:" + ($selection[1].name as string))
	new_toolpath.baseobject.renderable=true ;	new_toolpath.material = ToolpathMaterial ; new_toolpath.DisplayRenderMesh=false
	Global ToolPosition = addNewSpline new_toolpath
	TargetDepth = CutDepth
		
	if VectorCalcMethods[VectorCalcMethod] == "Angle from Reference Surface" or VectorCalcMethods[VectorCalcMethod] == "Tangent to Surface" then 
										   ( if not IsValidNode ReferenceMesh then ( messagebox "Pick Reference Surface from Vector Calculations Menu" ; return 0)
											tempReferenceMesh = copy ReferenceMesh ; ConvertToMesh tempReferenceMesh 
											Global VertexArray= #() ; VertexArray[tempReferenceMesh.numVerts] = undefined
											VertexArray = for i = 1 to VertexArray.count collect (GetVert tempReferenceMesh i)
											)
											
									
	AppendToolpath ZeroPoint.pos "Start/End"	
	if MakeVectors then setMaterialID new_toolpath ToolAngle 1 5 -- Vector	

	 for ShapeInSelection in Selection as Array do  						-- Number of shapes loop
	 ( 		

		for Pass = 1.0 to MultiPasses do
		( CutDepth = (Pass / MultiPasses) * TargetDepth ;	if DEBUG then format "CutDepth: %\n" CutDepth
			 
			if DEBUG then Format "Processing % \n" ShapeInSelection 
			if SuperClassOf ShapeInSelection == SHAPE then
			(	ShapeToProcess = Copy ShapeInSelection
				Convertto ShapeToProcess splineshape
				ResetXforms
				if DEBUG then format "ShapeToProcess: %\n" ShapeToProcess
				if pre_subdivide then Subdivide_Shape ShapeToProcess
			
				  for pns = 1 to numsplines ShapeToProcess do	-- Number of splines loop
				  (
					nk = numknots ShapeToProcess pns
					for pnk = 1 to nk do				-- Number of knots loop
						(k = getknotpoint ShapeToProcess pns pnk
						
							(
							AppendToolpath k "Position+Clearance"
							if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
							setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 3 -- Rapid
							
							AppendToolpath k "Plunge"
							if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
							setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 4 -- Plunge
							
							AppendToolpath k "Position+Clearance"						
							if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
							setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 2 -- Retract
							) 
								   
						 )
				   )
			   )
				delete ShapeToProcess
			  )
		) --ShapeInSelection
	
	AppendToolpath ZeroPoint.pos "Start/End"		
	try (new_toolpath.wirecolor = red) catch() 	
	if MakeVectors then setMaterialID  new_toolpath ToolAngle 1 5 -- Vector
	setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 3 -- Rapid	
	
	-- Finish off Toolpath
	-- AppendToolpath k 3
	if VectorCalcMethods[VectorCalcMethod] == "Angle from Reference Surface" or VectorCalcMethods[VectorCalcMethod] == "Tangent to Surface" then delete tempReferenceMesh	
	updateshape new_toolpath; select new_toolpath
	--setSplineSelection new_toolpath ToolAngles
  )
  
function AppendToolpath Position mode =(	--Add knot to Toolpath / Add Vectors to Toolpath
	if DEBUG then format "AppendToolpath - Position:% mode:% \n" Position mode 
	
	Vector = CalcToolAngleVector Position
	--format "Vector:%\n" Vector

	if TangentMode then 
		(
		-- Offset Tool by Tool radius
		Position += (ToolDiameter / 2) * normalize Vector 
		-- Rotate Vector 90
		VT = [0,0,0] ; VT.x = Vector.x ; VT.y = Vector.z ; VT.z = -Vector.y ; Vector = normalize VT
		)

	
	case mode of
	(
	"Position":				(	--Add Knot to Toolpath at toolpath position
							AddKnot new_toolpath ToolPosition #corner #line (Position + (Vector * CutDepth))
							if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
													addKnot new_toolpath ToolAngle #corner #line (Position + (Vector * CutDepth))
							  					 	addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (CutDepth + VectorSize) )
													)
							)
							
			
	"Retract":				(	--Add Knot to Toolpath at toolpath position + Clearance in Vector Direction
							AddKnot new_toolpath ToolPosition #corner #line (Position + Vector * (-CutDepth + tool_clearance))
							if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
													addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + tool_clearance))
							  				 		addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + tool_clearance + VectorSize ))
													)
							)	
											
							
	"Plunge":				(	--Add Knot to Toolpath at toolpath position in Plunge mode, ie. reduced Feedrate
							AddKnot new_toolpath ToolPosition #corner #line (Position + Vector * CutDepth)
							if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
													addKnot new_toolpath ToolAngle #corner #line Position 
							  					 	addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + VectorSize) )
													)
							)
							
	"Plunge/Dwell":			(	--Add Knot to Toolpath at toolpath position in "Plunge/Dwell" mode, ie. reduced Feedrate then Pause
							AddKnot new_toolpath ToolPosition #corner #line (Position + Vector * CutDepth)
							if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
													addKnot new_toolpath ToolAngle #corner #line Position 
							  					 	addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + VectorSize) )
													)
							)
						
						
			
	"Position+Clearance":	(	--Add Knot to Toolpath at toolpath position + Clearance in Vector Direction
							AddKnot new_toolpath ToolPosition #corner #line (Position + Vector * tool_clearance)
							if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
													addKnot new_toolpath ToolAngle #corner #line (Position + Vector * tool_clearance)
							  				 		addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (tool_clearance + VectorSize))
													)
							)	
				
	"Start/End":			(	--Add Knot to Toolpath at Start/End toolpath according to ClearanceMode
							if DEBUG then format "Start/End - ClearanceMode: % \n" ClearanceMode 
							case ClearanceMode of
								(
								1:	(	-- At Zero Point
									--AddKnot new_toolpath ToolPosition #corner #line (Position + Vector * tool_clearance)
									AddKnot new_toolpath ToolPosition #corner #line (Position)
								   	if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
															addKnot new_toolpath ToolAngle #corner #line (Position)
															addKnot new_toolpath ToolAngle #corner #line (Position + [0,0,VectorSize])
															)		
									)
												
								2:	(	-- At Zero Point + Clearance
									AddKnot new_toolpath ToolPosition #corner #line (Position + [0,0,tool_clearance])
								   	if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
														--	addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + tool_clearance))
															addKnot new_toolpath ToolAngle #corner #line (Position + [0,0,tool_clearance])
						  				 					addKnot new_toolpath ToolAngle #corner #line (Position + [0,0,tool_clearance] + [0,0,VectorSize])
															)	
									)						
									
								3:	(	-- At Spline Start / End 
									AddKnot new_toolpath ToolPosition #corner #line (Position + Vector * (CutDepth + tool_clearance))
									if MakeVectors then (	ToolAngle = addNewSpline new_toolpath; append ToolAngles ToolAngle
															addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + tool_clearance))
						  				 					addKnot new_toolpath ToolAngle #corner #line (Position + Vector * (-CutDepth + tool_clearance + VectorSize ))
															)					
									)						
								) -- end case
							)	-- End -- Add Knot to Toolpath at Start/End toolpath according to ClearanceMode
	)
)
function CalcToolAngleVector Position =(	
	if DEBUG then format "CalcToolAngleVector - Position:% VectorSize:% \n" Position VectorSize
	if DEBUG then format "VectorCalcMethod: %\n" VectorCalcMethods[VectorCalcMethod]

	case VectorCalcMethods[VectorCalcMethod] of
	(
	"Don't Calculate Angles":			Vector = [0,0,1]
	
	"Angle is Parallel to Z-axis":		Vector = [0,0,1]								
		
	"Angle from Origin":					----Use Direction of Toolpath Point from 0,0,0 to approximate Toolpath direction
												Vector = Normalize Position
									
	"Angle Around X-Axis":				----Use Direction of Toolpath Point around the X-Axis to approximate Toolpath direction
												Vector = Normalize [0,Position.y,Position.z]
		
	"Angle from Reference Spline":	----Use Direction of closest point on ReferenceSpline to approximate Toolpath direction
												Vector = ( TangentCurve3D ReferenceSpline 1 (nearestpathparam ReferenceSpline 1 [ReferenceSpline.pos.x,Position.y,Position.z])) * quat 90 [-1,0,0]
		
	"Angle from Reference Surface":	----Use Direction of closest point on ReferenceMesh to approximate Toolpath direction
												Vector = GetClosestVertexVector tempReferenceMesh Position
	)
	if DEBUG then format "VectorPoint: %\n" VectorPoint 
	return Vector
)
function GetClosestVertexVectorOLD MeshObj coords =(  -- OldMethod, slower
	VertexArray= #{1..MeshObj.numVerts} ; local closest = 999999999, closestIdx, dist
for i in VertexArray do 
	( dist=(distance coords (GetVert MeshObj i)) 
	if dist < closest then (closest = dist;	closestIdx = i)
	) 
	Vector = GetNormal MeshObj closestIdx
	--ProjectionTape= tape pos:(coords + (( GetNormal MeshObj closestIdx) *10000000)) target:(targetObject pos:coords)
	--try ( Vector = (intersectRay MeshObj (ProjectionTape as ray)).dir ) catch(Vector = [0,0,1])
	--Delete ProjectionTape
	return Vector
)
function GetClosestVertexVectorNearestVertex MeshObj coords =( -- Faster method - Create Array of vertex coordinates once at start of MakeToolpath and perform calcs on that.
local closest = 999999999, closestIdx, dist
for i = 1 to VertexArray.count do 
	( dist=(distance coords VertexArray[i]) 
	if dist < closest then (closest = dist;	closestIdx = i)
	) 
	Vector = GetNormal MeshObj closestIdx
)
function GetClosestVertexVector MeshObj coords =( -- Most efficient method for objects lying on XAxis.  Project coords towards X-axis and return Normal Vector of intersection with Mesh Object
	ProjectionTape= tape pos:(coords+ (coords * [0,1.5,1.5])) target:(targetObject pos:[coords.x,0,0])
	try ( Vector = (intersectRay MeshObj (ProjectionTape as ray)).dir ) catch(Vector = [0,0,1])
	Delete ProjectionTape
	Return Vector
)
function Detach_Splines =(  ----Splits shape into its component splines
Detached_Splines=#()
Global Original_shape=selection[1]
	if classof Original_shape != splineshape then copy Original_shape; Converttosplineshape Original_shape				
		for pns = 1 to numSplines Original_shape do																
		(	tmp_shape = splineShape name:("Detached_Spline"+ pns as string); tmp_shape.adaptive=true; tmp_shape.wirecolor=random black white 
			tmp_spline = addNewSpline tmp_shape
			nk = numknots Original_shape pns 											

			for pnk = 1 to nk do															
				(				
				knot_pos1=getknotpoint  Original_shape pns pnk
				in_vec_pos1=getInVec  	Original_shape pns pnk
				out_vec_pos1=getOutvec  Original_shape pns pnk		
				addknot tmp_shape tmp_spline #beziercorner #curve knot_pos1 in_vec_pos1 out_vec_pos1
				if pnk==nk and isClosed Original_shape pns then close tmp_shape 1
				)-- number knots in spline
				updateshape tmp_shape ;			append Detached_Splines tmp_shape
		)-- number splines in shape
set1= selectionsets ["Detached_Splines"+Original_Shape.name] = Detached_Splines
delete Original_shape
)
function Is_Curved_Seg Shape Spline Seg=(	---------  Find if a particular segment of a shape is curved as defined by Max_Curvature, returns true or fals
---------  also returns false if lenght between points is below the threshold defined by min_Seg_length
	if Seg < numKnots Shape Spline then
	(	Knot_pos1=getKnotpoint  Shape Spline Seg
		Knot_pos2=getKnotpoint  Shape Spline (Seg+1)
		out_vec_pos1=getOutvec  Shape Spline Seg
		in_vec_pos2=getInVec  	Shape Spline (Seg+1)
	)	else	(	Knot_pos1=getKnotpoint  Shape Spline Seg
					Knot_pos2=getKnotpoint  Shape Spline 1					
					out_vec_pos1=getOutvec  Shape Spline Seg
					in_vec_pos2=getInVec  	Shape Spline 1
				)
	
	if distance Knot_pos1 Knot_pos2 < min_Seg_length then return false 
	if distance Knot_pos1 Knot_pos2 > max_Seg_length then return true 
	
	v1= normalize 	( Knot_pos1 - Knot_pos2 )
	v2= normalize 	( Knot_pos1 - out_vec_pos1 )
	v4= normalize 	( Knot_pos2 - in_vec_pos2 )
	Curviness = length( cross v1 v2 ) + length( cross v1 v4 ) ;	-- format "Curviness %  (Max_Curvature/100) % \n" Curviness  (Max_Curvature as float/100)
	if Curviness >= (Max_Curvature as float/100) then return true else return false
)
function Subdivide_Shape Shape =(	--Divide segments of shape which are particularly curved
	if DEBUG then Format "Subdividing %\n" Shape
	StartTimer2()
	for pass = 1 to subdivide_recursions do
	(	
		for Spline = 1 to numsplines Shape do
		(	Seg_to_do=0
			for Seg = 1 to numsegments Shape Spline do
			(
			Seg_to_do += 1
				if Is_Curved_Seg shape spline Seg_to_do then 
				(
				refineSegment Shape spline Seg_to_do 0.5
				--updateShape Shape-- changed at 4.30e to speed things up.
				Seg_to_do += 1
				)
			)	
		)
	)
updateShape Shape
if DEBUG then format "Finished Initial Sub-Division of: %\n" Shape
EndTimer2 "Subdivide_Shape()"
)
function SubdivideAndRenameShape =( 
Global Selected_Shape = copy $ prefix: ("SubDivided_" + ($.name as string))  ---- make a copy of shape
if classof Selected_Shape != splineshape then Converttosplineshape Selected_Shape
Subdivide_Shape Selected_Shape
Select Selected_Shape
)
function Make_Contour_Lines =(
	try(
	Global non_contour_obj = #()	Global contour_obj = #()
	Global contour_lines_array = #()
	Global tmp_shape
	
	for viz_obj= 1 to selection.count do contour_obj[viz_obj] = selection[viz_obj]
	max select invert
	for hide_objs = 1 to selection.count do non_contour_obj[hide_objs] = selection[hide_objs]
	hide selection
	select (contour_obj)
	for obj in contour_obj do convertToMesh obj 
	contour_lines = splineShape prefix: "Contour"
	minZ = (selection.min.z) + Dist_from_base ;	maxZ = (selection.max.z) - Dist_from_top  
	
	for currentZ = maxZ to minZ by -Spacing do -- start loop... 
	( 
	s = section pos:[0, 0, currentZ] -- create Section 
	max views redraw 
	tmp_shape=convertToSplineShape s -- convert Section to SplineShape 
	append contour_lines_array tmp_shape
	) 
	
	unhide non_contour_obj
) 
catch ()
)
function Attach_Splines =(	
		disableSceneRedraw()
		tmp_shape = splineShape prefix:"Attached_Splines"; tmp_shape.wirecolor=random black white 
		for i = 1 to contour_lines_array.count do
		( 	spline_to_attach=contour_lines_array[i]
			for pns = 1 to numSplines spline_to_attach do
				(	tmp_spline = addNewSpline tmp_shape
					for pnk = 1 to numknots spline_to_attach pns do															
					(	knot_pos1=getknotpoint  spline_to_attach pns pnk
						addknot tmp_shape tmp_spline #corner #line knot_pos1
						if pnk==nk and isClosed spline_to_attach pns then close tmp_shape tmp_spline
					)-- number knots
				)-- number splines			
		)
		updateshape tmp_shape; select tmp_shape
		delete contour_lines_array
		enableSceneRedraw(); redrawViews()	
)												
function Create_Vertex_Array =( --creates a vertex array from selected shape
	Global vertex_array= #(#(),#(),#())
		ns=numsplines tmp_shape
		for pns= 1 to ns do
				(	nk= numknots tmp_shape pns
							for pnk=1 to nk do
							(		vertex_pos1=getknotpoint tmp_shape pns pnk
									append vertex_array [1]pns
									append vertex_array [2]pnk
									append vertex_array [3]vertex_pos1
							)
				)
)
function Closest_Vertex =(--	creates an array of closest vertices -- based on a Swami Script
		Global closest_vertex_array = #(#(),#(),#())
		Global vc = vertex_array[1].count
		Local a, b, dist_1, dist_min
		for a = 1 to vc do
			( pos_1= vertex_array[3][a]
				Local dist_min = 1e+38
				for b= 1 to vc do
					(				--  compare vertex 'a' with every other vertex 'b'
					pos_2 = vertex_array[3][b]
					dist_1 = distance pos_1 pos_2
						if dist_1 > 0 and dist_1 < dist_min and dist_1 < Fuse_n_Weld_threshold then
						( 	dist_min=dist_1 
							closest_vertex_array[1][a]=a		--vertex 'a' index number
							closest_vertex_array[2][a]=b		--vertex 'b' index number
							closest_vertex_array[3][a]=dist_min --distance from a to b
						)
					)			 		
			)
)
function Check_Vertices =(-- checks that there isn't another vertex closer
	for c= 1 to closest_vertex_array[1].count do 
	(	if (closest_vertex_array[3][c]) != undefined and (closest_vertex_array[3][c]) < Fuse_n_Weld_threshold then --- just processes relevant vertices
		(	dd= (closest_vertex_array[2][c])		
			if (closest_vertex_array[3][c]) > (closest_vertex_array[3][dd]) then closest_vertex_array[3][c]= 0.0
		)
	)
)
function Fuse_Vertices =(	-- moves a vertex and its closest neighbour to the average position of the pair
	for ii=1 to vc do
		( 
			if (closest_vertex_array[3][ii]) != undefined and (closest_vertex_array[3][ii]) > 0 and (closest_vertex_array[3][ii]) < Fuse_n_Weld_threshold then 
			(
			ave_pos = ((vertex_array [3][ii]) + (vertex_array [3][(closest_vertex_array[2][ii])]))/2
			setknotpoint tmp_shape vertex_array[1][ii] vertex_array[2][ii] ave_pos
			updateshape tmp_shape
			)
		)
)
function Weld_Vertices =(	
			setCommandPanelTaskMode mode:#modify
			subObjectLevel = 1
			max select all
			updateshape tmp_shape
			splineOps.weld tmp_shape
			subObjectLevel = 0	
			updateshape tmp_shape
)
Function ProjectSpline Surface Offset Method =( 
	if DEBUG then Format "ProjectSpline % % %\n" Surface Offset Method 
	if $ == undefined then (messageBox "Select Splines / Shapes to Project" ; return 0)
	Spline = $
	if SuperClassOf Spline != SHAPE then (messageBox "Select Splines / Shapes to Project" ; return 0)
	
	if Surface == undefined then (messageBox "Select Surface to Project onto" ; return 0)

	tempReferenceMesh = copy Surface
	Select tempReferenceMesh 
	if DEBUG then Format "tempReferenceMesh %\n" tempReferenceMesh
	ResetXforms
	Select Spline 
	ResetXforms	
	if DEBUG then Format "Spline %\n" Spline

	
Local NewVertPos = [0,0,0], IntersectDir = [0,0,0], IntersectPos = [0,0,0], NewVertPos, VertPosition, NewShape, IntersectDistance

ProjectionTape = tape pos:[0,0,0] target:(targetObject pos:[0,0,0] )
	
    for ShapeInSelection in Selection as Array do  						-- Number of shapes loop
    ( 	
		if DEBUG then Format "Processing % \n" ShapeInSelection 
		if SuperClassOf ShapeInSelection == SHAPE then
		(		
			ShapeToProcess = Copy ShapeInSelection
			Convertto ShapeToProcess splineshape
			--collapseStack ShapeToProcess
			format "ShapeToProcess: %\n" ShapeToProcess
			if ProjectionPreSubdivide then Subdivide_Shape ShapeToProcess
			NewShape =  splineShape prefix: "Projected"; 
			

			for SplineNumber = 1 to NumSplines ShapeToProcess do
			(	Local nk = numknots ShapeToProcess SplineNumber
				NewSpline = addNewSpline NewShape 
				for VertNumber = 1 to nk do
				(
				VertPosition = getknotpoint ShapeToProcess SplineNumber VertNumber

				case Method of
					( 	"Downwards": (
						ProjectionTape.pos = VertPosition ; ProjectionTape.target.pos = VertPosition - [0,0,10]
						)
						"AroundX": (
						ProjectionTape.pos = VertPosition ; ProjectionTape.target.pos = [VertPosition.x,0,0]
						)
					)
						
				ir = intersectRay tempReferenceMesh  (ProjectionTape as ray)
				
				if ir != undefined then	
					(
					IntersectPos = ir.pos ; IntersectDir = ir.dir 
					IntersectDirYZ = normalize [0,IntersectDir.y,IntersectDir.z] 
					
					IntersectDistance = Distance IntersectPos VertPosition
					if IntersectDistance < offset then  NewVertPos = VertPosition else
					NewVertPos = ( IntersectPos + ( IntersectDirYZ * offset ))
					) else NewVertPos = VertPosition 
					
					AddKnot NewShape NewSpline #corner #line NewVertPos
					
					
									if  VertNumber == nk then 
										( if (isclosed ShapeToProcess SplineNumber) then	 
											(
												VertNumber = 1  
												VertPosition = getknotpoint ShapeToProcess SplineNumber VertNumber

											case Method of
												( 	"Downwards": (
													ProjectionTape.pos = VertPosition ; ProjectionTape.target.pos = VertPosition - [0,0,10]
													)
													"AroundX": (
													ProjectionTape.pos = VertPosition ; ProjectionTape.target.pos = [VertPosition.x,0,0]
													)
												)
													
											ir = intersectRay tempReferenceMesh  (ProjectionTape as ray)
											
											if ir != undefined then	
												(
												IntersectPos = ir.pos ; IntersectDir = ir.dir 
												IntersectDirYZ = normalize [0,IntersectDir.y,IntersectDir.z] 
												
												IntersectDistance = Distance IntersectPos VertPosition
												if IntersectDistance < offset then  NewVertPos = VertPosition else
												NewVertPos = ( IntersectPos + ( IntersectDirYZ * offset ))
												) else NewVertPos = VertPosition 
												
												AddKnot NewShape NewSpline #corner #line NewVertPos
											)
						
										)
					
				)--Vert
				
			)--Spline
)--Shape
Delete ShapeToProcess
)
UpdateShape NewShape; Select NewShape
Delete ProjectionTape
Delete tempReferenceMesh 
RedrawViews()
)
function SplineFromMesh MeshObject method:1 toolpath:0 =(
MeshObject = copy MeshObject
Convertto MeshObject mesh
Global new_toolpath = splineShape prefix: "Toolpath", ToolPosition = addNewSpline new_toolpath, ToolAngles = #()
LastVerts = #(0,0)

	case method of
(	1:	(	----Vertex Method
			for Verx = 1 to MeshObject.numverts do 
			( 	VertPosition = getVert MeshObject Verx
				AddKnot new_toolpath ToolPosition #corner #line VertPosition
				if toolpath == 1 then
				(
				ToolAngle = addNewSpline new_toolpath
				AddKnot new_toolpath ToolAngle #corner #line VertPosition
				AddKnot new_toolpath ToolAngle #corner #line (VertPosition + VectorSize *( GetNormal MeshObject Verx))
				setMaterialID  new_toolpath ToolAngle 1 5
				setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 1
				) 
			)
		)
	2:	(	----Edge Method, Optimized
			for edge in MeshObject.edges do 
			(
			Verts = meshop.getVertsUsingEdge MeshObject edge 
			for Vert in Verts do if Verx != LastVerts[1] and Verx != LastVerts[2] then 
				(	Verx = Vert
					VertPosition = getVert MeshObject Verx
					AddKnot new_toolpath ToolPosition #corner #line VertPosition
					if toolpath == 1 then
					(
					ToolAngle = addNewSpline new_toolpath
					AddKnot new_toolpath ToolAngle #corner #line VertPosition
					AddKnot new_toolpath ToolAngle #corner #line (VertPosition + VectorSize *( GetNormal MeshObject Verx))
					setMaterialID  new_toolpath ToolAngle 1 5
					setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 1
					) 
				)
			LastVerts = verts as array
			)
		)		
	3:	(	----Faces Method
			for Face in MeshObject.Faces do
			(	
			verts = meshop.getVertsUsingFace MeshObject  Face 
			for vert in verts do 
				( 	Verx = vert
					VertPosition = getVert MeshObject Verx
					AddKnot new_toolpath ToolPosition #corner #line VertPosition
					if toolpath == 1 then
					(
					ToolAngle = addNewSpline new_toolpath
					AddKnot new_toolpath ToolAngle #corner #line VertPosition
					AddKnot new_toolpath ToolAngle #corner #line (VertPosition + VectorSize *( GetNormal MeshObject Verx))
					setMaterialID  new_toolpath ToolAngle 1 5
					setMaterialID  new_toolpath 1 (numsegments new_toolpath 1) 1
					) 
				)
			)
		)
)


	delete MeshObject 
	updateshape new_toolpath; select new_toolpath
)

-- POST PROCESS --
function PostProcessToolpath =(   
	if DEBUG then format "PostProcessToolpath\n" 
	if Toolpath == undefined or classof Toolpath != splineshape then (messageBox "Select a Toolpath to Process" ; return 0)
	if DEBUG then Format "Toolpath: %\n" Toolpath
	-- Check if valid Toolpath
	if Toolpath.material.name != "Toolpath" then (messageBox "Pick a Valid Toolpath\n (Invalid Material Assigned)" ; return 0)
	if numsplines Toolpath != 1 and (numsplines Toolpath != (numknots Toolpath 1) + 1) then (messageBox "Pick a Valid Toolpath\n (Invalid Number of Splines in Toolpath)" ; return 0)
	if numsplines Toolpath == 1 then ( ToolpathHasVectors = False ;format "Toolpath without Vectors\n" )
	if numsplines Toolpath == (numknots Toolpath 1) + 1 then ( ToolpathHasVectors = True ;format "Toolpath with Vectors\n" ) 
	--ResetXForms()
	
	Global MoveType = "Feed Move", LastMoveType = "Feed Move", MatID = 0, LastMatID = 0
	Global MachineCoordsStart= SixAxisCoords [0.0,0.0,0.0] [0.0,0.0,0.0]
	Global MachineCoords   = SixAxisCoords [0.0,0.0,0.0] [0.0,0.0,0.0]
	Global LastAngleA=0.0, RevolutionsA=0, LastAngleB=0.0, RevolutionsB=0, LastAngleC=0.0, RevolutionsC=0, ToolpathAngleA=0.0, ToolpathAngleB=0.0, ToolpathAngleC=0.0 
	Global XCoords, YCoords, ZCoords, ACoords, BCoords, LineNo
	Global NearestAngleAZero = 0.0, NearestAngleBZero = 0.0, NearestAngleCZero = 0.0
	Global X=0.0, Y=0.0, Z=0.0, A=0.0, B=0.0, C=0.0
	Global DegreesToAZero = 0, DegreesToBZero = 0, DegreesToCZero = 0, CutterAngles = [0,0,0]

	-- Array of distances travelled by Tool Tip; KnotDistances = getSegLengths Toolpath 1 byVertex:true 
	
	SceneSetup()
	Cutter.pos = ZeroPoint.pos
	MachineCoordsStart.pos = Cutter.pos	
	if DEBUG then format "MachineCoordsStart.pos %\n" MachineCoordsStart.pos 
	case MachineTypes[MachineType] of
			(
			"XYZ"				: (Print "XYZ" 			; CalcToolPathExportCoordsXYZ())
			"XZA"				: (Print "XZA" 			; CalcToolPathExportCoordsXYZA())	
			"XYZA"			: (Print "XYZA" 		; CalcToolPathExportCoordsXYZA())
			"XYZAB"			: (Print "XYZAB" 		; CalcToolPathExportCoordsXYZAB())
			"XYZBC"			: (Print "XYZBC" 		; CalcToolPathExportCoordsXYZBC())
			"XYZAC"			: (Print "XYZAC"		; CalcToolPathExportCoordsXYZAC())
			"Robot Arm"	: (Print "Robot Arm"	; CalcToolPathExportCoordsRobotArm())	
			)
)
function GetMoveType Coord =(	-- Find the type of move through use of Material ID
	try (MatID = getMaterialID Toolpath 1 (Coord - 1)) catch MatID = 3
	--MatID = getMaterialID $ 1 2
	if DEBUG then Format "MatID: %\n" MatID
	if LastMatID 	!= 1 then MoveType = "Start of Feedmove"
	if MatID 		== 1 then MoveType = "Feed Move"
	if MatID 		== 2 then MoveType = "Retract Move"
	if MatID 		== 3 then MoveType = "Rapid Move"
	if MatID 		== 4 then MoveType=  "Plunge Move"
	if MatID 		== 6 then MoveType=  "Plunge/Dwell"
	LastMatID 		= MatID
	if Debug then format "MoveType: %\n" MoveType
	Return MoveType 
)
function GetToolVector Coord =(
	ToolpathVector = normalize ( ( getknotpoint Toolpath (Coord + 1) 2 ) - ( getknotpoint Toolpath (Coord + 1) 1))
	if abs ToolpathVector.x < 0.00001 then ToolpathVector.x = 0 ; if abs ToolpathVector.y < 0.00001 then ToolpathVector.y = 0 ; if abs ToolpathVector.z < 0.00001 then ToolpathVector.z = 0
	return ToolpathVector 
)
function CalcToolPathExportCoordsXYZ =(
	if DEBUG then Format "fn CalcToolPathExportCoordsXYZ\n"

	WriteToolpath "start"
	MachineCoordsStart.pos = Cutter.pos
	local nk = numknots Toolpath 1
	deleteKeys Toolpath ; animationrange = (interval 1 nk) ; animate on
	
	for pnk= 1 to nk do at time pnk 
		(  	
			if DEBUG then format "pnk: %\n" pnk
			Cutter.pos = getknotpoint Toolpath 1 pnk ----Get Position of Point on Toolpath 	
			MachineCoords.pos = Cutter.pos - MachineCoordsStart.pos 
			GetMoveType pnk
			WriteToolpath "coords"
		)
	WriteToolpath "end"
	SliderTime = 0f
)
function CalcToolPathExportCoordsXYZA = (
	WriteToolpath "start"
	local nk = numknots Toolpath 1	
	deleteKeys Toolpath ; animationrange = (interval 1 nk) ; animate on
  
	for pnk= 1 to nk do at time pnk 
		(  	
			if DEBUG then Format "Point: % \n" pnk
			Toolpath.dir = [0,0,1]
			ToolpathVector = GetToolVector pnk 
			ToolpathAngleA = atan2 ToolpathVector.y ToolpathVector.z
			Toolpath.rotation = quat ToolpathAngleA x_axis
			AAxis.rotation = quat ToolpathAngleA x_axis
			Cutter.pos = getknotpoint Toolpath 1 pnk
			MachineCoords.pos = Cutter.pos - MachineCoordsStart.pos 			
			
			--- Adjust ToolpathAngleA to account for number of previous turns of A-Axis ----
			if DEBUG then format "LastAngleA: % \nRevolutions: % \n " LastAngleA RevolutionsA  
		 	DegreesToAZero = ToolpathAngleA
			if ToolpathAngleA >  180 then DegreesToAZero =  360 - ToolpathAngleA 
			if ToolpathAngleA < -180 then DegreesToAZero = -360 - ToolpathAngleA 
			if RevolutionsA  != 0 then ToolpathAngleA += 360 * RevolutionsA
			AngleDiff = ToolpathAngleA - LastAngleA
			if AngleDiff >  180 then ( ToolpathAngleA -= 360; RevolutionsA -= 1 )
			if AngleDiff < -180 then ( ToolpathAngleA += 360; RevolutionsA += 1 )
			LastAngleA= ToolpathAngleA
			NearestAngleAZero = ToolpathAngleA - DegreesToAZero
			if DEBUG then format "NearestAngleAZero % \n" (RoundOffNumber NearestAngleAZero) 
			
			MachineCoords.ang.x = ToolpathAngleA
			
			GetMoveType pnk
			WriteToolpath "coords"
		)
	WriteToolpath "end"
	SliderTime = 0f
)
function CalcToolPathExportCoordsXYZAB = (
	MachineCoordsStart.pos = BAxis.pos	
	WriteToolpath "start"
	local nk = numknots Toolpath 1	
	deleteKeys Toolpath ; animationrange = (interval 1 nk) ; animate on
  
	for pnk= 1 to nk do at time pnk 
		(  	
			if DEBUG then Format "Point: % \n" pnk
			Toolpath.dir = [0,0,1]
			ToolpathVector = GetToolVector pnk 
			
			-- find AAxis Angle and rotate toolpath
			ToolpathAngleA = atan2 ToolpathVector.y ToolpathVector.z
			Toolpath.rotation = quat ToolpathAngleA x_axis
			AAxis.rotation = quat ToolpathAngleA x_axis
							
			--- Adjust ToolpathAngleA to account for number of previous turns of A-Axis ----
			if DEBUG then format "LastAngleA: % \nRevolutions: % \n " LastAngleA RevolutionsA  
		 	DegreesToAZero = ToolpathAngleA
			if ToolpathAngleA >  180 then DegreesToAZero =  360 - ToolpathAngleA 
			if ToolpathAngleA < -180 then DegreesToAZero = -360 - ToolpathAngleA 
			if RevolutionsA  != 0 then ToolpathAngleA += 360 * RevolutionsA
			AngleDiff = ToolpathAngleA - LastAngleA
			if AngleDiff >  180 then ( ToolpathAngleA -= 360; RevolutionsA -= 1 )
			if AngleDiff < -180 then ( ToolpathAngleA += 360; RevolutionsA += 1 )
			LastAngleA= ToolpathAngleA
			NearestAngleAZero = ToolpathAngleA - DegreesToAZero
			if DEBUG then format "NearestAngleAZero % \n" (RoundOffNumber NearestAngleAZero) 
			
			-- find BAxis Angle
			Cutter.pos = getknotpoint Toolpath 1 pnk
			ToolpathVector = GetToolVector pnk
			
			ToolpathAngleB = -(atan2 ToolpathVector.x ToolpathVector.z)
			Cutter.dir = [0,0,1]
			in coordsys local Cutter.rotation = quat -90 z_axis
			in coordsys local Cutter.rotation = quat ToolpathAngleB x_axis
				
			--Cutter.dir = ToolpathVector
			--if Cutter.dir == [0,0,1] then in coordsys local Cutter.rotation = quat -90 z_axis
			--ToolpathAngleB = -1 * ((quatToEuler (inverse BAxis.rotation)).y) 	
			--ToolpathAngleB = (quatToEuler (inverse BAxis.rotation)).y	
			
			MachineCoords.ang.x = ToolpathAngleA
			MachineCoords.ang.y = -ToolpathAngleB	
			MachineCoords.ang.z = 0.0
			MachineCoords.pos = BAxis.pos - MachineCoordsStart.pos 
			
			GetMoveType pnk
			WriteToolpath "coords"
		)
	WriteToolpath "end"
	SliderTime = 0f
)
function CalcToolPathExportCoordsXYZBC = (	
	MachineCoordsStart.pos = CAxis.pos	
	WriteToolpath "start"
	local nk = numknots Toolpath 1	
	deleteKeys Toolpath ; animationrange = (interval 1 nk) ; animate on
  
	for pnk= 1 to nk do at time pnk 
		(  	
			if DEBUG then Format "Point: % \n" pnk
			Toolpath.dir = [0,0,1]
			Cutter.dir = [0,0,1]
			Cutter.pos = getknotpoint Toolpath 1 pnk ----Get Position of Point on Toolpath 	
			
			ToolpathVector = GetToolVector pnk 
			
			ToolpathAngleC = (atan2 ToolpathVector.y ToolpathVector.x)
			if DEBUG then format "ToolpathAngleC1  %\n" ToolpathAngleC 
			if ToolpathAngleC <= 0 then ToolpathAngleC += 180			
			if DEBUG then format "ToolpathAngleC2  %\n" ToolpathAngleC
			if ToolpathAngleC == 180 then ToolpathAngleC = 0			
	
			Cutter.dir = ToolpathVector
			if DEBUG then format "Cutter.dir %\n" Cutter.dir
			if Cutter.dir == [0,0,1] then in coordsys local Cutter.rotation = quat -90 z_axis
			--in coordsys local BAxis.rotation = quat -90 z_axis

			MachineCoords.ang = (quatToEuler (inverse BAxis.rotation))
			ToolpathAngleB = MachineCoords.ang.y
			
			if DEBUG then format "ToolpathAngleB %\n" ToolpathAngleB 		
			
			Cutter.dir = [0,0,1]
				
			in coordsys local Cutter.rotation = quat (ToolpathAngleC - 90) z_axis
			in coordsys local Cutter.rotation = quat -ToolpathAngleB x_axis		
		--	in coordsys local Cutter.rotation = quat -90 z_axis

			MachineCoords.ang.z = ToolpathAngleC 
			
			MachineCoords.pos = CAxis.pos - MachineCoordsStart.pos 			
			
			GetMoveType pnk
			WriteToolpath "coords"
			
			
		)
	WriteToolpath "end"
	SliderTime = 0f
)
function CalcToolPathExportCoordsXYZAC = (	
	if DEBUG then format "function CalcToolPathExportCoordsXYZAC:\n"
	MachineCoordsStart.pos = CAxis.pos	
	WriteToolpath "start"
	local nk = numknots Toolpath 1	
	deleteKeys Toolpath ; animationrange = (interval 1 nk) ; animate on
  
	for pnk= 1 to nk do at time pnk 
		(  	Toolpath.dir = [0,0,1] ; Cutter.dir = [0,0,1]
			if DEBUG then Format "Point: % \n" pnk
			Cutter.pos = getknotpoint Toolpath 1 pnk ----Get Position of Point on Toolpath 	
			ToolpathVector = normalize (GetToolVector pnk)
			if abs ToolpathVector.x < 0.00001 then ToolpathVector.x = 0.00000001 ; if abs ToolpathVector.y < 0.00001 then ToolpathVector.y = 0 ; if abs ToolpathVector.z < 0.00001 then ToolpathVector.z = 0
			if DEBUG then format "ToolpathVector: %\n" ToolpathVector
			
			Cutter.dir = ToolpathVector ; if DEBUG then format "Cutter.dir %\n" Cutter.dir			
			CutterAngles = (quatToEuler (inverse Cutter.rotation));	if DEBUG then format "Cutter Angles; A % B % C % \n"  CutterAngles.x CutterAngles.y CutterAngles.z 
			
			ToolpathAngleA = CutterAngles.x ; if DEBUG then format "ToolpathAngleA %\n" ToolpathAngleA				
			ToolpathAngleC = CutterAngles.z ; if DEBUG then format "ToolpathAngleC1  %\n" ToolpathAngleC 
			--ToolpathAngleC = (atan2 ToolpathVector.y ToolpathVector.x) ; if DEBUG then format "ToolpathAngleC1  %\n" ToolpathAngleC 
		
			-- Quadrant
			--if ToolpathVector.y >0 and ToolpathVector.x >0 then format "Quad 1\n"
			--if ToolpathVector.y <0 and ToolpathVector.x >0 then format "Quad 2\n"
			--if ToolpathVector.y <0 and ToolpathVector.x <0 then format "Quad 3\n"
			--if ToolpathVector.y >0 and ToolpathVector.x <0 then format "Quad 4\n"
			
			--if ToolpathAngleC <= 0 then ToolpathAngleC += 180			
			--if DEBUG then format "ToolpathAngleC2  %\n" ToolpathAngleC
			--if ToolpathAngleC == 180 then ToolpathAngleC = 0			

	
			--ToolpathAngleC -= 90
			--if ToolpathVector = [0,0,1] then ToolpathAngleC = 0
			if ToolpathVector.x == 0 then ToolpathAngleC = 0
			
		--	if ToolpathVector.x > 0 then ToolpathAngleC = -360 + CutterAngles.z
			
		--		(
		--		--format "LastAngleC: % \n RevolutionsC: % \n " LastAngleC RevolutionsC  
		--	 	DegreesToCZero = ToolpathAngleC
		--		if ToolpathAngleC >  180 then DegreesToCZero =  360 - ToolpathAngleC 
		--		if ToolpathAngleC < -180 then DegreesToCZero = -360 - ToolpathAngleC 
		--		if RevolutionsC  != 0 then ToolpathAngleC += 360 * RevolutionsC
		--		AngleDiff = ToolpathAngleC - LastAngleC
		--		if AngleDiff >  180 then ( ToolpathAngleC -= 360; RevolutionsC -= 1 )
		--		if AngleDiff < -180 then ( ToolpathAngleC += 360; RevolutionsC += 1 )
		--		LastAngleC= ToolpathAngleC
		--		) else 	in coordsys local Cutter.rotation = quat -90 z_axis

			Cutter.dir = [0,0,1]
			
		--	in coordsys local rotate Cutter (eulerangles 0 0 (- ToolpathAngleC))
			
			if ToolpathVector != [0,0,1] then (in coordsys local Cutter.rotation = quat (ToolpathAngleC - 90) z_axis) else (in coordsys local Cutter.rotation = quat -90 z_axis)
			in coordsys local Cutter.rotation = quat ToolpathAngleA y_axis
			
			MachineCoords.ang.x = ToolpathAngleA
			MachineCoords.ang.y = 0
			MachineCoords.ang.z = ToolpathAngleC 
			MachineCoords.pos = CAxis.pos - MachineCoordsStart.pos 			
			GetMoveType pnk
			WriteToolpath "coords"
		)
	WriteToolpath "end"
	SliderTime = 0f
)
function CalcToolPathExportCoordsRobotArm = (
	Format "fn CalcToolPathExportCoordsRobotArm\n"
	--enableSceneRedraw()

	WriteToolpath "start"

	for pnk= 1 to animationrange.end do at time pnk 
		(  	
			if DEBUG then format "pnk: %\n" pnk
			SliderTime = pnk
			WriteToolpath "coords"
		)
	WriteToolpath "end"
	SliderTime = 0f
)
function CalcToolPathExportCoords =(  		
	local nk= numknots Toolpath 1
	if DEBUG then Format "Number of Points on Toolpath: % :% \n" nk toolpath
	deleteKeys Toolpath ; animationrange = (interval 1 nk) ; animate on
	
	WriteToolpath "start"
	
  	for pnk= 1 to nk do at time pnk   
	( --Format "\n% \n" pnk
		Toolpath.dir = [0,0,1]
		Cutter.dir = [0,0,1]
		in coordsys local Cutter.rotation = quat -90 z_axis
		
		ToolpathPoint  = getknotpoint Toolpath 1 pnk ----Get Position of Point on Toolpath 	
		Cutter.pos = ToolpathPoint
	
		if ToolpathHasVectors then
				(	
					ToolpathVector = normalize ( ( getknotpoint Toolpath (pnk + 1) 2 ) - ( getknotpoint Toolpath (pnk + 1) 1))
					if abs ToolpathVector.x < 0.00001 then ToolpathVector.x = 0 ; if abs ToolpathVector.y < 0.00001 then ToolpathVector.y = 0 ; if abs ToolpathVector.z < 0.00001 then ToolpathVector.z = 0
				--	format "Vector %\n" ToolpathVector 
				--	Cutter.dir = ToolpathVector 
					
			-----	if A is operating as a standard 4th axis, ie. rotating the workpiece then rotate the whole toolpath to bring A to zero, have the cutter dir straight up.
					if FindString ExportAxesMethodString "A" != undefined then 
					(
						ToolpathAngleA = atan2 ToolpathVector.y ToolpathVector.z
						-- format "ToolpathAngleA : %\n" ToolpathAngleA
						Toolpath.rotation = quat ToolpathAngleA x_axis
					 	Cutter.pos = getknotpoint Toolpath 1 pnk ----Get Position of Point on Toolpath 	
		
						--- Adjust ToolpathAngleA to account for number of previous turns of A-Axis ----
						--- format "LastAngleA: % \nRevolutions: % \n " LastAngleA RevolutionsA  
					 	DegreesToAZero = ToolpathAngleA
						if ToolpathAngleA >  180 then DegreesToAZero =  360 - ToolpathAngleA 
						if ToolpathAngleA < -180 then DegreesToAZero = -360 - ToolpathAngleA 
						if RevolutionsA  != 0 then ToolpathAngleA += 360 * RevolutionsA
						AngleDiff = ToolpathAngleA - LastAngleA
						if AngleDiff >  180 then ( ToolpathAngleA -= 360; RevolutionsA -= 1 )
						if AngleDiff < -180 then ( ToolpathAngleA += 360; RevolutionsA += 1 )
						LastAngleA= ToolpathAngleA
						NearestAngleAZero = ToolpathAngleA - DegreesToAZero
						--	Format "NearestAngleAZero % \n" (RoundOffNumber NearestAngleAZero) 
					)
			
	
					ToolpathVector = normalize ( ( getknotpoint Toolpath (pnk + 1) 2 ) - ( getknotpoint Toolpath (pnk + 1) 1))
					if abs ToolpathVector.x < 0.00001 then ToolpathVector.x = 0 ; if abs ToolpathVector.y < 0.00001 then ToolpathVector.y = 0 ; if abs ToolpathVector.z < 0.00001 then ToolpathVector.z = 0
					
					ToolpathAngleB = (atan2 ToolpathVector.x ToolpathVector.z)
				--	format "ToolpathAngleB : %\n" ToolpathAngleB
	
					Cutter.dir = ToolpathVector
					CutterAngles = (quatToEuler (inverse Cutter.rotation))
				--	format "Cutter Angle; B % A % C % \n"  CutterAngles.x CutterAngles.y CutterAngles.z 
					--ToolpathAngleB = CutterAngles.x
					--ToolpathAngleC = CutterAngles.z
			
			----	if using the B-axis
					if FindString ExportAxesMethodString "B" != undefined then 
						(
						--format "ToolpathAngleB : %\n" ToolpathAngleB
						Cutter.dir = [0,0,1]
						in coordsys local Cutter.rotation = quat ToolpathAngleB y_axis
						)
						
			----- 	if using C-Axis then Adjust ToolpathAngleC to account for number of previous turns of C-Axis --------
					if FindString ExportAxesMethodString "C" != undefined then 
						(
						--format "LastAngleC: % \n RevolutionsC: % \n " LastAngleC RevolutionsC  
					 	DegreesToCZero = ToolpathAngleC
						if ToolpathAngleC >  180 then DegreesToCZero =  360 - ToolpathAngleC 
						if ToolpathAngleC < -180 then DegreesToCZero = -360 - ToolpathAngleC 
						if RevolutionsC  != 0 then ToolpathAngleC += 360 * RevolutionsC
						AngleDiff = ToolpathAngleC - LastAngleC
						if AngleDiff >  180 then ( ToolpathAngleC -= 360; RevolutionsC -= 1 )
						if AngleDiff < -180 then ( ToolpathAngleC += 360; RevolutionsC += 1 )
						LastAngleC= ToolpathAngleC
						) else 	in coordsys local Cutter.rotation = quat -90 z_axis
	
				) 
			--MachineCoords.pos = CAxis.pos - ZeroPoint.pos + [OffsetAroundCaxis,0,-OffsetAroundBaxis]
			MachineCoords.pos = CAxis.pos - MachineCoordsStart.pos
			--Format "MachineCoords: %\n" MachineCoords.pos
		
			-------------------------------------------------------------------------------------------------
			MachineCoords.line.x= pnk
			MachineCoords.ang.x = ToolpathAngleA
			MachineCoords.ang.y = ToolpathAngleB
			MachineCoords.ang.z = ToolpathAngleC
			-------------------------------------------------------------------------------------------------
		
			if ExportAxesMethodString == "XYZAC" then
			(
			MachineCoords.pos.x = CAxis.pos.x - ZeroPoint.pos.x + OffsetAroundBaxis
			MachineCoords.pos.y = CAxis.pos.y - ZeroPoint.pos.y - OffsetAroundCaxis
			MachineCoords.pos.z = CAxis.pos.z - ZeroPoint.pos.z 
			MachineCoords.ang.x += 90
			MachineCoords.ang.y = 0
			MachineCoords.ang.z += 90
			MachineCoords.ang.z *= -1
			)
		
			--format "% ToolPivot \t ZeroPoint % OffsetAroundBaxis%\n" ToolPivot.pos ZeroPoint.pos OffsetAroundBaxis
			--format "CAxis Pos: %\n" CAxis.pos
		
			WriteToolpath "coords"
			)
WriteToolpath "end"
)
function WriteToolpath type =(	
	if DEBUG then format "fn WriteToolpath\n" OutputString 

	OutputString = ToolpathExportString type
	if DEBUG then format "OutputString: %\n" OutputString 
	
	case ExportDestinations[ExportDestination] of 
	(
	"File":
		(	case type of
			(
			"start": 	( ----------- save file dialogue
					    sve = getsavefilename caption:"Save to:" types:"G_Code (*.iso)|*.iso"
					    outfile = getfilenamefile sve;	outpath = getfilenamepath sve ; outfile += ".iso"
						format "Saving to File:\t% \n" (outpath + "\\" + outfile)
					   	Global ToolpathExportFile = createfile (outpath + "\\" + outfile)
						format OutputString to:ToolpathExportFile 
						)
			"coords": 	( format OutputString to:ToolpathExportFile )
						
			"end":		( format OutputString to:ToolpathExportFile; close ToolpathExportFile; format "Finished Exporting to file.\n")
			)
		)
		
	"Printer/Serial Port":
		(	case type of			(
			"start": 	(Global ToolpathExportFile = createfile DefaultFileName
						 if DEBUG then format "Exporting to Temp File: %\n" DefaultFileName 
						 format OutputString to:ToolpathExportFile
						)
						
			"coords": 	(format OutputString to:ToolpathExportFile )
			
			"end":		(format OutputString to:ToolpathExportFile 
						close ToolpathExportFile ;
						format "Finished Exporting to Temp file., %\n" DefaultFileName 
		
							case OutputPort of 
							(
							1: 	(--LPT1	
								--DOSCommand  ("COPY " + DefaultFileName + " LPT1" )
								DOSCommand ( "print /d:LPT2 " + DefaultFileName )
								)
							2: 	(--LPT2	
								DOSCommand  ("COPY " + DefaultFileName + " LPT2" )
								)
							3: 	(--COM1	
								DOSCommand ( "MODE COM1:9600,N,8,1,P" )
								DOSCommand  ("COPY " + DefaultFileName + " COM1" )	
								)
							4: 	(--COM2	
								DOSCommand ( "MODE COM2:9600,N,8,1,P" )
								DOSCommand  ("COPY " + DefaultFileName + " COM2" )
								)	
							)	
						)
			)
		)
		
	"Default File":
		(	case type of
			(
			"start": 	( Global ToolpathExportFile = createfile DefaultFileName
						  if DEBUG then  format "Exporting to Default File: %\n" DefaultFileName 
						  format OutputString to:ToolpathExportFile
						)
			"coords": 	( format OutputString to:ToolpathExportFile )
			"end":		( format OutputString to:ToolpathExportFile ; close ToolpathExportFile ; format "Finished Exporting to file.\n")
			)
		)		
	"Script Listener Window":
		(	case type of
			(
			"start": 	(ClearListener(); format "( Output Starts Here )\n" ; format outputString )
			"coords": 	(format OutputString )
			"end":		(format OutputString ; format "\n( Output Ends Here )\n" )
			)
		)
	)
-- ExportString

)-------------------------------------------------------------------------------------------------
function ToolpathExportString type = (	
	if  DEBUG then format "fn ToolpathExportString, type: %\n" type 
	OutputString = ""

	if ZDirection == 2 then MachineCoords.pos.z *= -1
	XCoords = (RoundOffNumber MachineCoords.pos.x) as string 
	YCoords = (RoundOffNumber MachineCoords.pos.y) as string 
	ZCoords = (RoundOffNumber MachineCoords.pos.z) as string 
	ACoords = (RoundOffNumber MachineCoords.ang.x) as string 
	BCoords = (RoundOffNumber MachineCoords.ang.y) as string 
	CCoords = (RoundOffNumber MachineCoords.ang.z) as string 
	if (MoveType == "Plunge Move") or (MoveType == "Plunge/Dwell") then F = (RoundOffNumber (Feedrate * (PlungeRatio / 100.0)) places:2) as string else F = (RoundOffNumber Feedrate places:2) as string 
	

	--LineNo  = (MachineCoords.line.x as integer) as string
	
	case ExportFormats[ExportFormat] of
	(
	"Standard G-Code":
		(	case type of
			(
			"start":	( 
						if DEBUG then format "Start of Standard G-Code Format:\n"
							OutputString += "(File:" + maxFileName +  ")" + "\n"
						if (MachineTypes[MachineType] == "Robot Arm") 	then 
							(OutputString += "Robot Mode " + "\n") 
								else
									(OutputString += "(" + Toolpath.name +  ")" + "\n")
							if not SubRoutine then
							(
							OutputString += "T1M6 \n"
							OutputString += "M3 \n" 
							) else
							( -- Define variables #1 for Feedrate and #2 for PlungeRate for use in subroutines
							OutputString +=	"(Define variables #1 for Feedrate and #2 for PlungeRate for use in subroutines) \n"
							OutputString += "#1 = " + F + " \n"
							OutputString += "#2 = " + (RoundOffNumber (Feedrate * (PlungeRatio / 100.0)) places:2) as string + " \n" 
							)
						
						)
						
			"coords":	( 
					--	if DEBUG_Level2 then format "G0andG1state: %\n" G0andG1state
						if OutputLineNo then OutputString += "N" + LineNo + " "
						if MoveType != LastMoveType then
						(	
							if not SubRoutine then
							(
							case MoveType of
									(
									"Start of Feedmove"	:(OutputString += "G1 "; OutputString += "F" + F as string + " ")
									"Feed Move"			:(OutputString += "G1 "; OutputString += "F" + F as string + " ")
									"Retract Move"		:(OutputString += "G0 ")
									"Rapid Move"			:(OutputString += "G0 ")
									"Plunge Move"			:(OutputString += "G1 "; OutputString += "F" + F as string + " ")
									"Plunge/Dwell"			:(OutputString += "G1 "; OutputString += "F" + F as string + " ")
									)
							)
							else -- Use variables #1 for Feedrate and #2 for PlungeRate in subroutines
							(
							case MoveType of
									(
									"Start of Feedmove"	:(OutputString += "G1 F#1 ")
									"Feed Move"			:(OutputString += "G1 F#1 ")
									"Retract Move"		:(OutputString += "G0 ")
									"Rapid Move"			:(OutputString += "G0 ")
									"Plunge Move"			:(OutputString += "G1 F#2 ")
									"Plunge/Dwell"			:(OutputString += "G1 F#2 ")
									)
							)
						) else if MoveType == "Feed Move" and RecalcFeedrateEnabled then OutputString += "F" + F + " "
						LastMoveType = MoveType
							
						-- Fixed Z - this is where the Z-axis is either up or down to preset positions.
						if FixedZ then
							(
							  case MoveType of
								(
								"Start of Feedmove"	:(ZCoords = FixedZDown)
								"Feed Move"			:(ZCoords = FixedZDown)
								"Retract Move"		:(ZCoords = FixedZUp)
								"Rapid Move"			:(ZCoords = FixedZUp)
								"Plunge Move"			:(ZCoords = FixedZDown)
								"Plunge/Dwell"			:(ZCoords = FixedZDown)
								)
							)
						-- /Fixed Z --
							
							case MachineTypes[MachineType] of
							(
							"XYZ":		(
							   					OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + "\n"
												)
									
							"XYA":		(
												YCoords = (-1 * Z) as string 
												ACoords = RoundOffNumber (ToolpathAngleA - 90) as string 
												OutputString += "X" + XCoords  + " Y"+ YCoords + " A" + ACoords + "\n"
												)
							
							"XZA":		(
							   					OutputString += "X" + XCoords  + " Z" + ZCoords + " A" + ACoords + "\n"
												)													
								
							"XYZA":		(
												OutputString += "X" + XCoords + " Y"+ YCoords + " Z" + ZCoords + " A" + ACoords  + "\n"
												)
								
							"XYZAB":	(
												OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + " A" + ACoords + " B" + BCoords + "\n"												
												)	
								
							"XYZBC":	(
		 										OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + " B" + BCoords + " C" + CCoords + "\n"
												)
												
							"XYZAC":	(
		 										OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + " A" + ACoords + " C" + CCoords + "\n"
												)
												
							"Robot Arm":	(
												OutputString += " X " +  $slider01.value as string  
												OutputString += " Y " +  $slider02.value as string 
												OutputString += " Z " +  $slider03.value as string 
												OutputString += " A " +  $slider04.value as string 
												OutputString += " B " +  $slider05.value as string 
												OutputString += " C " +  $slider06.value as string 
												OutputString += " D " +  $slider07.value as string 
												OutputString += "\n"
												)
												
												
							)
						if MoveType == "Plunge/Dwell" then OutputString += "G04 P" + (Dwell as String) + "\n"
						)
				
			"end":		(	
							if DEBUG then format "Ending Standard G-Code Format:\n"
							if FindString (MachineTypes[MachineType]) "A" != undefined then OutputString += "G92 A0\n"
							if not SubRoutine then
								(
								OutputString += "M5\n"
								OutputString += "M30\n"
								)
								else
								(
								OutputString += "M99\n"
								OutputString += "\n"
								)
								
						)
			)
		)	

	"Digitise": -- Use Digitise Probe routine following toolpath -- Simply use G31 Probe Move instead of Plunge Move		
		(	case type of
			(
			"start":	( 
						if DEBUG then format "Start of Standard G-Code Format:\n"
							OutputString += "(Digitise Mode)" + "\n"
							OutputString += "(File:" + maxFileName +  ")" + "\n"
							OutputString += "M40 \n" 
						if (MachineTypes[MachineType] == "Robot Arm") then (OutputString += "Robot Mode " + "\n") else (OutputString += "(" + Toolpath.name +  ")" + "\n")
						)
						
			"coords": ( 
					--	if DEBUG_Level2 then format "G0andG1state: %\n" G0andG1state
						if OutputLineNo then OutputString += "N" + LineNo + " "
						if MoveType != LastMoveType then
						(	
							case MoveType of
									(
									"Start of Feedmove"	:(OutputString += "G1 "; OutputString += "F" + F as string + " ")
									"Feed Move"			:(OutputString += "G1 "; OutputString += "F" + F as string + " ")
									"Retract Move"		:(OutputString += "G0 ")
									"Rapid Move"			:(OutputString += "G0 ")
									"Plunge Move"			:(OutputString += "G31 "; OutputString += "F" + F as string + " ")
									"Plunge/Dwell"			:(OutputString += "G31 "; OutputString += "F" + F as string + " ")
									)
						) else if MoveType == "Feed Move" and RecalcFeedrateEnabled then OutputString += "F" + F + " "
						LastMoveType = MoveType
							
						-- Fixed Z - this is where the Z-axis is either up or down to preset positions.
						if FixedZ then
							(
							  case MoveType of
								(
								"Start of Feedmove"	:(ZCoords = FixedZDown)
								"Feed Move"			:(ZCoords = FixedZDown)
								"Retract Move"		:(ZCoords = FixedZUp)
								"Rapid Move"			:(ZCoords = FixedZUp)
								"Plunge Move"			:(ZCoords = FixedZDown)
								"Plunge/Dwell"			:(ZCoords = FixedZDown)
								)
							)
						-- /Fixed Z --
							
							case MachineTypes[MachineType] of
							(
							"XYZ":		(
							   					OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + "\n"
												)
									
							"XYA":		(
												YCoords = (-1 * Z) as string 
												ACoords = RoundOffNumber (ToolpathAngleA - 90) as string 
												OutputString += "X" + XCoords  + " Y"+ YCoords + " A" + ACoords + "\n"
												)
							
							"XZA":		(
							   					OutputString += "X" + XCoords  + " Z" + ZCoords + " A" + ACoords + "\n"
												)													
								
							"XYZA":		(
												OutputString += "X" + XCoords + " Y"+ YCoords + " Z" + ZCoords + " A" + ACoords  + "\n"
												)
								
							"XYZAB":	(
												OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + " A" + ACoords + " B" + BCoords + "\n"												
												)	
								
							"XYZBC":	(
		 										OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + " B" + BCoords + " C" + CCoords + "\n"
												)
												
							"XYZAC":	(
		 										OutputString += "X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + " A" + ACoords + " C" + CCoords + "\n"
												)
												
							"Robot Arm":	(
												OutputString += " X " +  $slider01.value as string  
												OutputString += " Y " +  $slider02.value as string 
												OutputString += " Z " +  $slider03.value as string 
												OutputString += " A " +  $slider04.value as string 
												OutputString += " B " +  $slider05.value as string 
												OutputString += " C " +  $slider06.value as string 
												OutputString += " D " +  $slider07.value as string 
												OutputString += "\n"
												)
												
							)
						if MoveType == "Plunge/Dwell" then OutputString += "G04 P" + (Dwell as String) + "\n"
						)
				
			"end":		(	
							if DEBUG then format "Ending Standard G-Code Format:\n"
							if FindString (MachineTypes[MachineType]) "A" != undefined then OutputString += "G92 A0\n"
							OutputString += "M5\n"
							OutputString += "M30\n"
						)
			)
		)			
		
	"Standard HPGL":
		(	case type of
			(
			"start":	( 
						if DEBUG then format "Start of Standard HPGL Format:\n"
						OutputString += "IN;PU;PA;\n"
    					OutputString += "VS" + F + ";\n"
						OutputString += "SP1;\n"
						)
						
			"coords":	(if MoveType != LastMoveType then
									(	case MoveType of
										(
										"Retract Move": (OutputString += "PU"	) -- Rapid Move
										"Plunge Move": (OutputString += "PD"	) -- Feed Move
										)
									)	
						LastMoveType = MoveType 
						OutputString += XCoords + " " + YCoords + "\n"
						)
				
			"end":		(OutputString += "PU;PA0,0;SP0;")
			)
		)		
		
	"Roland CNC HPGL": -- Multiply mm's by 100
		(	case type of
			(
			"start": 	(   
							if DEBUG then format "Start of Roland CNC HPGL Format:\n"
							if SpindleSpped != 0 then OutputString += "^IN \n!MC1\n" else OutputString += "^IN \n!MC0\n"
							OutputString += "V " + F + " F " + F + "\n"
						)
			"coords": 	(	OutputString += "Z " + ((100 * RoundOffNumber MachineCoords.pos.x) as string) + ", "+ ((100 * RoundOffNumber MachineCoords.pos.y) as string)  + ", " + ((100 * RoundOffNumber MachineCoords.pos.z) as string)  + "\n"	)
			"end":		( 	OutputString += "!MC0\n")
			)
		)
		
	"HAPFO-CNC":
		(
						if ZDirection == 2 then MachineCoords.pos.z *= -1
						XCoords = ((RoundOffNumber MachineCoords.pos.x *1000) as integer) as string 
						ZCoords = (((RoundOffNumber MachineCoords.pos.z *-1000) as integer)) as string 
						ACoords = ((RoundOffNumber MachineCoords.ang.x *100) as integer) as string 
						F = ((RoundOffNumber Feedrate places:3) as integer * 1000) as string

			case type of
			(
			"start": 	(   
							if DEBUG then format "Start of HAPFO-CNC Format:\n"
							LineNo = 0
							OutputString += HAPFOLineNo() + "IMF_PBL HAPFO-CNC  4-AXES CNC\n"
							OutputString += HAPFOLineNo() + "; **************************************\n"
							OutputString += HAPFOLineNo() + "; MEGACAD WOOD-TURNING-SYSTEM V4.6\n"
							OutputString += HAPFOLineNo() + "; **************************************\n"
							OutputString += HAPFOLineNo() + "; NCP-CODE: RAB*.NCP DATE: 05.08.2002\n"
							OutputString += HAPFOLineNo() + "; **************************************\n"
							OutputString += HAPFOLineNo() + "; MILLING\n"
							OutputString += HAPFOLineNo() + "WPZERO 2 ; MILLING TOOL FRONT\n"
							OutputString += HAPFOLineNo() + "SETPORT A1=04D ; SPINDLE OFF\n"
							OutputString += HAPFOLineNo() + "SETPORT A1=08D ; Z-AXIS ON\n"
							OutputString += HAPFOLineNo() + "FASTABS Y-50000\n"
							OutputString += HAPFOLineNo() + "REWIND Z \n"
							OutputString += HAPFOLineNo() + "FASTABS X50000 Y-50000 Z0 ; CONTOUR NR: 1\n"	
						)
						
			"coords": 	(						
							if MoveType == "Rapid Move" then (	OutputString += HAPFOLineNo() + "FASTABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" + HAPFOLineNo() + "VEL " + F + "\n") -- Rapid Move
							else (	OutputString += HAPFOLineNo() + "MOVEABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" ) -- Feed Move
						
							if MoveType != LastMoveType then
							(	case MoveType of
								(
								"Start of Feedmove"	:(	OutputString += HAPFOLineNo() + "MOVEABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" ) -- Feed Move
								"Feed Move"			:(	OutputString += HAPFOLineNo() + "MOVEABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" ) -- Feed Move
								"Retract Move"		:(	OutputString += HAPFOLineNo() + "FASTABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" + HAPFOLineNo() + "VEL " + F + "\n") -- Rapid Move 
								"Rapid Move"		:(	OutputString += HAPFOLineNo() + "FASTABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" + HAPFOLineNo() + "VEL " + F + "\n") -- Rapid Move
								"Plunge Move"		:(	OutputString += HAPFOLineNo() + "MOVEABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" ) -- Feed Move
								)
							) else if MoveType == "Feed Move" and RecalcFeedrateEnabled then (	OutputString += HAPFOLineNo() + "MOVEABS" + " X" + XCoords + " Y"+ ZCoords + " Z" + ACoords + "\n" ) -- Feed Move
							LastMoveType = MoveType
						
						
						)
						"end":		( 	OutputString += HAPFOLineNo() + "FASTABS Y-50000\n"
							OutputString += HAPFOLineNo() + "REWIND Z \n"
							OutputString += HAPFOLineNo() + "FASTABS Y-50000\n"
							OutputString += HAPFOLineNo() + "SETPORT A1=08D\n"
							OutputString += HAPFOLineNo() + "VEL 10000\n"
							OutputString += HAPFOLineNo() + "PROGEND\n"
						)
			)
		)	

	"CSV": -- Spreadsheet

		(
			OutputString += XCoords + "," + YCoords + "," + ZCoords + "\n"
	
		)

	"CamBam": -- Spreadsheet

		(
			OutputString += "G1 X" + XCoords  + " Y"+ YCoords + " Z" + ZCoords + "\n"
	
		)			
		
	)

return OutputString
)

--User Interface--
fn CleanTPDialogs =( 	
	if DEBUG then format "Cleaning Dialogs\n"
	try (DestroyDialog MachineTypeDialog ) catch() 	;	try (MachineType_Rollout.b_SetupMachineType.checked = false) catch()
	try (DestroyDialog TPFShapeDialog ) catch() 	;	try (ToolpathGenerator_Rollout.b_ToolpathFromShape.checked = false) catch()
	try (DestroyDialog TPFShapeDialog ) catch() 	;	try (ToolpathGenerator_Rollout.b_DigitisePeckToolpath.checked = false) catch()
	try (DestroyDialog TPFProjdShapeDialog) catch()	;	try (ToolpathGenerator_Rollout.b_ProjectedShape.checked = false) catch()
	try (DestroyDialog TPFStepoversDia ) catch()	;	try (ToolpathGenerator_Rollout.b_StepoverToolpath.checked = false) catch()
	try (DestroyDialog TPFDrillCycleDia ) catch()	;	try (ToolpathGenerator_Rollout.b_DrillCycleToolpath.checked = false) catch()
				
	try (ToolpathGenerator_Rollout.b_ToolpathFromMesh.checked = false) catch()		
	try (ToolpathGenerator_Rollout.b_RingMaker.checked = false) catch()		
	try (ToolpathGenerator_Rollout.b_GearToothForm.checked = false) catch()		
	try (ToolpathGenerator_Rollout.b_ToolpathFromImage.checked = false) catch()		
)
rollout MachineType_rollout "Machine Type"   (
	Label L1 "-" align:#left offset:[5,0]
  	checkbutton b_SetupMachineType	"Config." width:50 height:20 offset:[60,-21]
	on b_SetupMachineType changed State do (CleanTPDialogs(); if State == true then (createDialog MachineTypeDialog 200 400 (CNCTKFloater.Pos.x + 215 ) CNCTKFloater.Pos.y ; b_SetupMachineType.checked = true))
	on MachineType_rollout open do MachineType_rollout.L1.text = MachineTypes[MachineType]
	
)
rollout MachineTypeDialog "Machine Type Setup"(
	radiobuttons RB_MachineType labels:MachineTypes Default:MachineType align:#left
 	on RB_MachineType changed state do ( 
										MachineType = RB_MachineType.state
										format "MachineType : %\n" MachineTypes[MachineType] 
										MachineType_rollout.L1.text = MachineTypes[MachineType]
										SceneSetup()
										)
	
	spinner s_OffsetAroundAaxis		"Offset around A axis:"   range:[0,1000,OffsetAroundAaxis] fieldwidth:40 type:#float offset:[-10,0]
	on s_OffsetAroundAaxis			changed val do (OffsetAroundAaxis= val ; SceneSetup() )
	
	spinner s_OffsetAroundBaxis		"Offset around B axis:"   range:[0,1000,OffsetAroundBaxis] fieldwidth:40 type:#float offset:[-10,0]
	on s_OffsetAroundBaxis			changed val do (OffsetAroundBaxis= val ; SceneSetup() )
	
	spinner s_OffsetAroundCaxis		"Offset around C axis:"   range:[0,1000,OffsetAroundCaxis] fieldwidth:40 type:#float offset:[-10,0]
	on s_OffsetAroundCaxis			changed val do (OffsetAroundCaxis= val ; SceneSetup() )
		
	radiobuttons RB_ZDirection labels:#("+Z moves away from work", "+Z moves towards work") Default:ZDirection align:#left offset:[0,10]
	on RB_ZDirection changed state do ZDirection = RB_ZDirection.state

	button b_LaunchCNCToolkitUI "Launch CNCToolkit" width:165 height:20 offset:[0,0]
	
	on b_LaunchCNCToolkitUI pressed do (LaunchCNCToolkitUI() ; DestroyDialog MachineTypeDialog )

	on MachineTypeDialog close do CleanTPDialogs()								
)
rollout ToolpathControl_rollout "Tool Control"   (
	spinner s_Feedrate 			"Feed Rate "  		range:[0.1,9999,Feedrate] fieldwidth:42 type:#float 
	on s_Feedrate 				changed val do Feedrate = val
    
	spinner s_PlungeRatio    	"Plunge Ratio % " 	range:[1,100,PlungeRatio] fieldwidth:42 type:#float
	on s_PlungeRatio   			changed val do PlungeRatio = val
	
	spinner s_tool_clearance  	"Tool Clearance "   range:[0,100,tool_clearance] fieldwidth:42 type:#float
	on s_tool_clearance 		changed val do (tool_clearance = val;  VectorSize = tool_clearance / 3)
	
	spinner s_CutDepth			"Cut Depth "   		range:[-100,0,CutDepth] fieldwidth:42 type:#float
	on s_CutDepth				changed val do CutDepth = val
	
	spinner s_Multipass			"MultiPass "   		range:[1,100,MultiPasses] fieldwidth:42 type:#integer
	on s_Multipass				changed val do MultiPasses = val
	
	spinner s_ToolDiameter		"Tool Diameter "   range:[0,1000,ToolDiameter] fieldwidth:42 type:#float
	on s_ToolDiameter			changed val do ToolDiameter = val

	spinner s_SpindleSpeed		"Spindle Speed "   range:[0,100000,SpindleSpeed] fieldwidth:42 type:#integer
	on s_SpindleSpeed			changed val do SpindleSpeed = val

	--spinner s_ToolOffset		"Tool Offset "   	range:[0,100,ToolOffset] fieldwidth:42 type:#float
	--on s_ToolOffset			changed val do ToolOffset = val
	
)
rollout TPFShapeDialog "Toolpath From Shape"(
	label L1 "Make Toolpath from the selected"
	label L2 "Shape, Curve or Line."

  	Button b_make_toolpath 	"Make Toolpath" 	width:129 height:20 offset:[0,20]
 	on b_make_toolpath     	pressed  do	( 	StartTimer1() ; SetWaitCursor(); gc() ; disableSceneRedraw()
											maketoolpath() 
											enableSceneRedraw(); redrawViews();	SetArrowCursor() ; EndTimer1 "Make Toolpath"
										)
										
	on TPFShapeDialog close do CleanTPDialogs()								
)
rollout TPFProjdShapeDialog "From Projected Shape" (

	label L1 "Make Toolpath from currently selected "
	label L2 "Shape projected onto an Object"
	
	radiobuttons RB_ProjectionMethod "Projection Method " labels:ProjectionMethods offset:[-3,3] Default:ProjectionMethod align:#left
	
	on RB_ProjectionMethod changed state do (ProjectionMethod = RB_ProjectionMethod.state ; format "ProjectionMethod: %\n" ProjectionMethods[ProjectionMethod] )	
	
	PickButton p_reference_mesh	"Pick Reference Object" width:180 height:20 tooltip: "Pick Mesh"  offset:[0,5]  filter: MeshFilter 
	on p_reference_mesh	picked obj do ( 	
										ReferenceMesh = obj
										p_reference_mesh.text = ReferenceMesh.name
										TPFProjdShapeDialog.p_reference_mesh.text = ReferenceMesh.name
									)

  	Button b_make_toolpath 	"Make Toolpath" 	width:129 height:20 offset:[0,20]
 	on b_make_toolpath     	pressed  do	( 	StartTimer1() ; SetWaitCursor(); gc() ; disableSceneRedraw()
										--	ProjectSpline
											maketoolpath() 
											enableSceneRedraw(); redrawViews();	SetArrowCursor() ; EndTimer1 "Make Toolpath"
										)
										
	on TPFProjdShapeDialog close do CleanTPDialogs()								
)
rollout TPFStepoversDia "From Stepover Paths" (
	label L1 "Make Toolpath from Stepover Paths "

	local PX=100, PY=100, PR=10
	Spinner s_Stepover	"Stepover Distance "   	range:[0,100,Stepover]  fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_PX		"X Dimention "   		range:[0,1000,100.0]  	fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_PY		"Y Dimention "   		range:[0,1000,100.0]  	fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_PR		"Spiral Radius "   		range:[0,100,10.0]  	fieldwidth:38 type:#Float offset:[5,0]
	on s_Stepover 		changed val do Stepover = val
	on s_PX 			changed val do PX= val
	on s_PY 			changed val do PY= val
	on s_PR 			changed val do PR= val
	
	label L2 "Method:" align:#left
	radiobuttons RB_StepoverMethod labels:StepoverMethods  Default:StepoverMethod align:#left
 	on RB_StepoverMethod changed state do StepoverMethod = RB_StepoverMethod.state

	label L3 "If Reference Object is used" align:#centre offset: [0,4]
	label L4 "Toolpath will conform to object" align:#centre offset: [0,-4]
	PickButton p_reference_mesh	"Pick Reference Object" width:180 height:20 tooltip: "Pick Mesh"  offset:[0,0]  filter: MeshFilter 
	on p_reference_mesh	picked obj do ( ReferenceMesh = obj
						p_reference_mesh.text = ReferenceMesh.name
						TPFStepoversDia.p_reference_mesh.text = ReferenceMesh.name
						)

  	Button b_make_toolpath 	"Make Toolpath" 	width:129 height:20 offset:[0,20]
 	on b_make_toolpath     	pressed  do	( 	StartTimer1() ; SetWaitCursor(); gc() ; disableSceneRedraw()
							MakeParallelSplines PX PY PR
							--maketoolpath() 
							enableSceneRedraw(); redrawViews();	SetArrowCursor() ; EndTimer1 "Make Toolpath"
						)
										
	on TPFStepoversDia close do CleanTPDialogs()								
)
rollout TPFDrillCycleDia "Drill Cycle Toolpath"(
	label L1 "Make Drill Cycle from"
	label L2 "selected object's"
	label L3 "center positions"	
	
	local PX=100, PY=100, PR=10
	Spinner s_DrillDepth 	"Drill Depth "   	range:[0,100,DrillDepth]  fieldwidth:38 type:#Float offset:[5,10]
	Spinner s_PeckDepth 	"PeckDepth  "   	range:[0,1000,PeckDepth]  fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_Dwell			"Dwell Time "   	range:[0,1000,Dwell]  	  fieldwidth:38 type:#Float offset:[5,0]
	on s_DrillDepth 		changed val do DrillDepth = val
	on s_PeckDepth 			changed val do PeckDepth = val
	on s_Dwell				changed val do Dwell = val
	
  	Button b_make_toolpath 	"Make Toolpath" 	width:129 height:20 offset:[0,20]
 	on b_make_toolpath     	pressed  do	( 	StartTimer1() ; SetWaitCursor(); gc() ; disableSceneRedraw()
											MakeDrillCycleToolPath() 
											enableSceneRedraw(); redrawViews();	SetArrowCursor() ; EndTimer1 "Make Toolpath"
										)
										
	on TPFDrillCycleDia close do CleanTPDialogs()								
)
rollout ToolpathGenerator_Rollout "Toolpath Generator"  (
	checkbutton b_ToolpathFromShape 	"Toolpath From Shape" 				width:165 height:20	 --highlightcolor:(color 200 255 200)
 	--checkbutton b_ProjectedShape		"From Projected Shape" 		width:165 height:20
	--checkbutton b_StepoverToolpath		"Stepover Toolpath" 			width:165 height:20
	checkbutton b_DrillCycleToolpath	"Drill Cycle Toolpath" 					width:165 height:20
	checkbutton b_DigitisePeckToolpath	"Digitise/Pecking Toolpath" 		width:165 height:20
	
	
	--checkbutton b_ToolpathFromMesh		"Toolpath From Mesh" 		width:165 height:20
	--checkbutton b_RingMaker				"Ring Maker" 					width:165 height:20
	--checkbutton b_GearToothForm		"Gear Tooth Form"			width:165 height:20
	--checkbutton b_ToolpathFromImage	"Toolpath From Image"		width:165 height:20
	
	
	on b_ToolpathFromShape 	changed State do (CleanTPDialogs(); if State == true then 
		( 	StartTimer1() ; SetWaitCursor(); gc() ; disableSceneRedraw()
			maketoolpath() 
			enableSceneRedraw(); redrawViews();	SetArrowCursor() ; EndTimer1 "Make Toolpath"
		)
	)
	
		on b_DigitisePeckToolpath changed State do (CleanTPDialogs(); if State == true then 
		( 	StartTimer1() ; SetWaitCursor(); gc() ; disableSceneRedraw()
			DigitisePeckToolpath() 
			enableSceneRedraw(); redrawViews();	SetArrowCursor() ; EndTimer1 "Make Toolpath"
		)
	)
	
	on b_ProjectedShape		changed State do 	(CleanTPDialogs(); if State == true then (createDialog TPFProjdShapeDialog 200 200 (CNCTKFloater.Pos.x + 215) CNCTKFloater.Pos.y ; b_ProjectedShape.checked = true))
 	on b_StepoverToolpath		changed State do 	(CleanTPDialogs(); if State == true then (createDialog TPFStepoversDia 200 330 (CNCTKFloater.Pos.x + 215 ) CNCTKFloater.Pos.y ; b_StepoverToolpath.checked = true ))	
 	on b_DrillCycleToolpath		changed State do 	(CleanTPDialogs(); if State == true then (createDialog TPFDrillCycleDia 200 200 (CNCTKFloater.Pos.x + 215 ) CNCTKFloater.Pos.y ; b_DrillCycleToolpath.checked = true ))	
											
 	on b_ToolpathFromMesh	changed State do	(CleanTPDialogs() )
 	on b_RingMaker				changed State do	(CleanTPDialogs() )		
 	on b_GearToothForm		changed State do	(CleanTPDialogs() )	
 	on b_ToolpathFromImage	changed State do	(CleanTPDialogs() )		

	on ToolpathGenerator_Rollout close do CleanTPDialogs()

)
rollout Waterline_rollout "Contour Toolpath" (	
	Spinner s_Dist_from_top			"Start dist. from top"		range:[0,1000,Dist_from_top]  	fieldwidth:35 type:#Float offset:[9,0]
	Spinner s_Dist_from_base		"Stop dist. from base"   	range:[0,1000,Dist_from_base]  	fieldwidth:35 type:#Float offset:[9,0]
	Spinner s_Spacing				"Contour Spacing "    		range:[0.1,1000,Spacing] 		fieldwidth:35 type:#Float offset:[9,0]
	Spinner s_Fuse_n_Weld_threshold	"Weld Threshold "   		range:[0,1000,Fuse_n_Weld_threshold] fieldwidth:35 type:#Float offset:[9,-3]
	
	on s_Dist_from_top				changed val do Dist_from_top= val	
	on s_Dist_from_base       		changed val do Dist_from_base= val
	on s_Spacing			        changed val do Spacing= val 	
	on s_Fuse_n_Weld_threshold		changed val do Fuse_n_Weld_threshold= val		

	Button  	b_contour			"Waterline Selected Mesh/s" 	width:142 height:20 offset:[0,-2]
	on b_contour	pressed do 		(	setWaitCursor()
									make_contour_lines()
									attach_splines()
									create_vertex_array()
									closest_vertex()
									check_vertices()
									fuse_vertices()
									weld_vertices()
									setArrowCursor()
									)
)
rollout Project_rollout "Project Spline" (
Local ProjectOffset = 0
Button b_ProjectDownwards 	"Project Downwards" width:129 height:20
Button b_ProjectAroundX 		"Project Around X Axis" width:129 height:20
Spinner s_ProjectOffset		"Height Offset  " range:[-100,100,0] fieldwidth:42 type:#float
on s_ProjectOffset			 	changed val do ProjectOffset = val
on b_ProjectDownwards 		pressed do ProjectSpline ReferenceMesh ProjectOffset "Downwards"
on b_ProjectAroundX 			pressed do ProjectSpline ReferenceMesh ProjectOffset "AroundX"

PickButton p_reference_mesh	"Pick Reference Surface" width:135 height:15 tooltip: "Pick Mesh"  filter: MeshFilter 
on p_reference_mesh	picked obj do 	(	ReferenceMesh = obj
										p_reference_mesh.text = obj.name
										--ToolAngleControl_rollout.p_reference_mesh.text = ReferenceMesh.name
									)	
									
checkbox c_ProjectionPreSubdivide 	" Subdivide Spline First" 	checked:ProjectionPreSubdivide 
on c_ProjectionPreSubdivide 		changed state 			do 	ProjectionPreSubdivide = state
								
)
rollout ToolpathResolution_rollout "Resolution Control"  (

	checkbox c_pre_subdivide				" Auto Increase Resolution" 	checked:pre_subdivide
	spinner s_min_Seg_length   				"Min Seg Length   "   			range:[0,100,min_Seg_length] fieldwidth:31 type:#Float
	spinner s_max_Seg_length   				"Max Seg Length   "   			range:[0,100,max_Seg_length] fieldwidth:31 type:#Float
	spinner s_Max_Curvature   				"Max Curvature % "   			range:[0,100,Max_Curvature] fieldwidth:31 type:#integer
    spinner s_recursions 						"Recursions "   					range:[0,10,subdivide_recursions] fieldwidth:31 type:#integer
	Button b_SubdivideAndRenameShape   "SubDivide Selected Shape" 	width:150 height:15 tooltip: "Apply to Selected Shape"

	on c_pre_subdivide			changed state do 	pre_subdivide = state	
	on s_min_Seg_length 		changed val 	do 	min_Seg_length = val
	on s_max_Seg_length 		changed val 	do  max_Seg_length = val
	on s_Max_Curvature 		changed val 	do 	Max_Curvature = val
	on s_recursions				changed val 	do 	subdivide_recursions = val
	on b_SubdivideAndRenameShape     	pressed 		do 	(gc(); SetWaitCursor(); StartTimer1(); SubdivideAndRenameShape(); EndTimer1 "SubdivideAndRenameShape()"; enableSceneRedraw(); redrawViews(); SetArrowCursor())

)
rollout ToolAngleControl_rollout "Angle Control"  (
	radiobuttons RB_VectorCalcMethod "Calculate Tool-Angle Method" labels:VectorCalcMethods offset:[-7,0] Default:VectorCalcMethod align:#left
	on RB_VectorCalcMethod changed state do 
										(
										VectorCalcMethod = RB_VectorCalcMethod.state 
										format "VectorCalcMethod: %\n" VectorCalcMethods[VectorCalcMethod] 
										if RB_VectorCalcMethod.state == 1 then MakeVectors = false else MakeVectors = true 
										)	

	PickButton p_reference_Spline "Pick Reference Spline" width:165 height:15 offset:[0,0] tooltip: "Pick Spline" filter: SplineFilter
	on p_reference_Spline picked obj do 
										(
									 	ReferenceSpline = Convertto obj splineshape 		
										p_reference_Spline.text = "Ref Spline = " + ReferenceSpline.name
										)
	

	PickButton p_reference_mesh	"Pick Reference Surface" width:165 height:15 offset:[0,0] tooltip: "Pick Mesh"  filter: MeshFilter 	
	on p_reference_mesh	picked obj do 	
										( 	
										ReferenceMesh = obj
										p_reference_mesh.text = "Ref Mesh = " + ReferenceMesh.name
										Project_rollout.p_reference_mesh.text = ReferenceMesh.name
										)
										
	checkbox c_TangentMode 	" Tangent Mode" 	checked:TangentMode 
	on c_TangentMode changed state do TangentMode = state
									
)
rollout SplineFromMesh_Rollout "Spline from Mesh"   (
radiobuttons SplineFromMeshMethod labels:#("Vertex","Edge","Face") Default:1
Button b_SplineFromMesh		"Spline from Mesh" width:129 height:20
Button b_ToolpathFromMesh	"Toolpath from Mesh" width:129 height:20
on b_SplineFromMesh pressed do (
														StartTimer1() ; SetWaitCursor() ; gc() ; disableSceneRedraw()
														SplineFromMesh $ method:SplineFromMeshMethod.state
														enableSceneRedraw() ; redrawViews() ; SetArrowCursor() ; EndTimer1 "SplineFromMesh"
													)
													
on b_ToolpathFromMesh pressed do (
														StartTimer1() ; SetWaitCursor() ; gc() ; disableSceneRedraw()
														SplineFromMesh $ method:SplineFromMeshMethod.state toolpath:1 
														enableSceneRedraw() ; redrawViews() ; SetArrowCursor() ; EndTimer1 "ToolpathFromMesh"
														)
)
rollout PostProcess_Rollout "Post Process"   (
	Button b_PostProcess "Post Process Toolpath"  width:129 height:20
	on b_PostProcess pressed do 	
						(
						Global Toolpath = $
						StartTimer1() ; SetWaitCursor() ; gc() ; disableSceneRedraw()
						if (MachineTypes[MachineType] == "Robot Arm") then CalcToolPathExportCoordsRobotArm() else
						PostProcessToolpath()
						enableSceneRedraw() ; redrawViews() ; SetArrowCursor() ; EndTimer1 "Post Process"
						try (select Toolpath) catch()
						)
)
rollout PostProcessorOptions_Rollout "Post Processor Options" (
	--checkbox CB_RecalcFeedrateEnabled	"Recalc Feedrate per move" checked:RecalcFeedrateEnabled offset:[-9,0] tooltip:"Recalculate the Feedrate based on the distance that the toolpoint moves for 5/5 axis moves" 
	--on CB_RecalcFeedrateEnabled changed state do RecalcFeedrateEnabled = state

		radiobuttons RB_ExportFormat "Export Format" labels:ExportFormats Default:ExportFormat align:#left offset:[0,0]
		on RB_ExportFormat changed state do ExportFormat = RB_ExportFormat.state

		radiobuttons RB_ExportDestination "Destination" labels:ExportDestinations Default:ExportDestination align:#left
		on RB_ExportDestination changed state do ExportDestination = RB_ExportDestination.state
	
		listbox  RB_OutputPort "Output Port" items:#("LPT 1","LPT 2","COM 1","COM 2") selection:OutputPort height:2 align:#left
		on RB_OutputPort changed state do OutputPort = RB_OutputPort.selection
		
		label PostProcessorOptions_Lab_2 "Default File Name:" align:#left
		edittext ET_DefaultFileName ""  fieldWidth:200 offset: [-11,0] text: DefaultFileName
		on ET_DefaultFileName changed txt do DefaultFileName = txt
)
rollout ToolpathImport_Rollout 	"Toolpath Import"  (
	
	Button b_importGCode  		"Import G_Code"   			width:140 height:15 tooltip: "Get Toolpath from File"
	Button b_import_Roland   		"Import Roland HPGL"  		width:140 height:15 tooltip: "Get Toolpath from File"
	Button b_import_PLC   		"Import PLC"   					width:140 height:15 tooltip: "Get Toolpath from File"	
	Button b_import_CSV   		"Import CSV"   				width:140 height:15 tooltip: "Get Toolpath from File"	

	on b_importGCode   		pressed do 	(
																StartTimer1() ; SetWaitCursor() ; gc() ; disableSceneRedraw()
																ImportGCODE()
																enableSceneRedraw() ; redrawViews() ; SetArrowCursor() ; EndTimer1 "ImportGCODE()"
																)
												
	on b_import_Roland   		pressed do ImportRolandToolpath()
	on b_import_PLC    			pressed do ImportPLC()
	on b_import_CSV   			pressed do ImportCSV()
)
rollout Utilities_Rollout "Utilities"(
	Button b_DetachSplines		"Detach Splines"  		width:140 height:15 tooltip: "Detach all splines in selected shape to individual shapes" 
	Button b_preview_toolpath  	"Preview Tool Motion"  	width:140 height:15 tooltip: "Preview Path Followed by Tool"
	Button b_SplineFromObjects 	"Spline From Objects"  	width:140 height:15 tooltip: "Create a Spline from a Selection of Objects, useful for getting a toolpath from digitised points"
  	Button b_MakeTextRing   		"Make Text Ring "  		width:140 height:15 tooltip: "MakeTextRing "
  	Button b_ResetXForms   		"Reset XForms   "  		width:140 height:15 tooltip: "ResetXForms "
  	Button b_Subdivide_Shape   	"Subdivide Shape"  		width:140 height:15 tooltip: "Subdivide_Shape"
	
	on b_DetachSplines  			pressed do detach_Splines()
	on b_preview_toolpath 		pressed do create_preview_toolpath()	
	on b_SplineFromObjects 		pressed do SplineFromObjects()
	on b_MakeTextRing  			pressed do MakeTextRing()
	on b_ResetXForms     			pressed do ResetXForms()
	on b_Subdivide_Shape  		pressed do (gc(); SetWaitCursor(); SubdivideAndRenameShape(); enableSceneRedraw(); redrawViews(); SetArrowCursor())
)
rollout About_Rollout "About" (
label lab1 "CNC Toolkit Beta"
label lab1B "By Rab Gordon"
label lab1C "email: rab@rainnea.com"
label lab1d "for updates, or visit:"
label lab1e "www.rainnea.com/cnc.htm"
label lab2 "How to Use:"
label lab2b "1: Create Splines / Shapes" align:#left offset:[-5,0]
label lab2c "2: Check Zero Point position"align:#left offset:[-5,0]
label lab2d "3: Select Splines for Toolpath"align:#left offset:[-5,0]
label lab3b "4: Check All Options"align:#left offset:[-5,0]
label lab4 "5: Click 'Make Toolpath'"align:#left offset:[-5,0]
label lab5 "6: Click 'Export Toolpath'"align:#left offset:[-5,0]
label lab7 "Check GCode Before Use." offset:[-5,0]
label lab7b "No Responsibility accepted" offset:[-5,0]
label lab7c "for use of this program" offset:[-5,0]
)
rollout Parallel_rollout "Parallel Splines"  (	
	local ParallelMethod=1, Stepover=1, PX=100, PY=100, PR=10
	Spinner s_Stepover	"Stepover Distance "   	range:[0,100,Stepover]  fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_PX			"X Dimention "   		range:[0,10000,100.0]  	fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_PY			"Y Dimention "   		range:[0,10000,100.0]  	fieldwidth:38 type:#Float offset:[5,0]
	Spinner s_PR			"Radius "   			range:[0,1000,10.0]  	fieldwidth:38 type:#Float offset:[5,0]
	on s_Stepover 		changed val do Stepover = val
	on s_PX 				changed val do PX= val
	on s_PY 				changed val do PY= val
	on s_PR 				changed val do PR= val
	
	label L_ParallelMethod "Method:" align:#left
	radiobuttons RB_ParallelMethod labels:#("X (hoizontal lines)", "Y", "Spiral", "Radial (rotary)", "Helix   (rotary)") Default:1 align:#left
 	on RB_ParallelMethod changed state do ParallelMethod = RB_ParallelMethod.state
	Button b_ParallelSplines "Make Parallel Splines" width:129 height:20 tooltip: "Make Parallel Splines"
	on b_ParallelSplines pressed do MakeParallelSplines ParallelMethod Stepover PX PY PR
)
function LaunchCNCToolkitUI = (
try (closerolloutfloater CNCTKFloater) catch()
CleanTPDialogs()

CNCTKFloater = newrolloutfloater "CNC Toolkit Beta" 200 600
  --if not GMAX then addrollout Waterline_Rollout CNCTKFloater rolledUp:true
 
addrollout MachineType_rollout CNCTKFloater rolledUp:false
addrollout ToolpathControl_rollout CNCTKFloater rolledUp:false
addrollout ToolpathResolution_rollout CNCTKFloater rolledUp:true
addrollout ToolAngleControl_Rollout CNCTKFloater rolledUp:true
  
  
  addrollout Parallel_Rollout CNCTKFloater rolledUp:true
  addrollout Project_Rollout CNCTKFloater rolledUp:true
--  addrollout SplineFromMesh_Rollout CNCTKFloater rolledUp:true

  addrollout ToolpathGenerator_Rollout CNCTKFloater rolledUp:false
  addrollout PostProcess_Rollout CNCTKFloater rolledUp:false
  addrollout PostProcessorOptions_Rollout CNCTKFloater rolledUp:true
  addrollout ToolpathImport_Rollout CNCTKFloater rolledUp:false
  addrollout Utilities_Rollout CNCTKFloater rolledUp:true
 -- addrollout About_Rollout CNCTKFloater rolledUp:false
 -- forceCompleteRedraw()
 
  					
if IsValidNode ReferenceMesh do	(
				Project_rollout.p_reference_mesh.text =  "Ref Mesh = " + ReferenceMesh.name
				ToolAngleControl_rollout.p_reference_mesh.text =  "Ref Mesh = " + ReferenceMesh.name
				)
 
)
----------------------------------------------------------------------

-- INITIALISE AND RUN --
EnableSceneRedraw(); RedrawViews(); ClearListener()
if DEBUG == undefined then Global DEBUG = false
InitialiseVariables()
CleanTPDialogs()
LaunchCNCToolkitUI()
CreateZeroPoint()
EnableSceneRedraw(); RedrawViews(); gc()
