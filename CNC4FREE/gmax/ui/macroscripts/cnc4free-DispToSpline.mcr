macroScript DispToSpline
category:"cnc4free"

(
ClearListener()
undo off

--Rabs Timer

fn StartTimer = (Global startTime = TimeStamp() ; format "Starting Timer: %\n" startTime)
fn EndTimer =
(	
	local midNiteAdj, dt, dtMin, dtSec, endTime, milliseconds
	
	endTime = TimeStamp()
	format "Ending Timer: %\n" endTime
		
	midNiteAdj=if startTime<endTime then 0 else 86400000	-- Handle midnite spanning.
	milliseconds =(endTime-startTime+midNiteAdj)
	
	dt = milliseconds/1000.

	if dt>=60 then
	(	-- We are here if dt is one minute or more.
		dtMin=(dt/60.) as integer
		dtSec=(mod dt 60) as integer
	)	else	( dtMin=0 ; dtSec=dt ) -- Less than a Minute
	
	-- RoundOffNumber off the seconds.
	dtSec*=10^2 ; dtSec= (dtSec+0.5) as integer ; dtSec/=((10^2) as float)
	format "Timed at % milliseconds (% Min, % sec) \n"  milliseconds dtMin dtSec 
)


------------------------------------------------------------------------------------------------------

-- create empty array to store vertex locations

Sarrg = #()

-- grab vertex locations from selected mesh and draw the spline

fn SVerts =
	(
	StartTimer()
	if selection.count !=1 then(
		messagebox "Incorrect object Selection.  Please select a single Mesh object"
		)else
		if (superclassof $ == GeometryClass) then (

			n = getNumVerts $.mesh
			Sarrg = for v = 1 to n collect
			(getVert $.mesh v)
			)else
		(messagebox "Please Select A Mesh object"
	
		)

-- Set position to start spline draw

	pos1 = Sarrg [1]

-- Start Drawing the spline

	new_splineshape = SplineShape pos:pos1 steps:0
	new_spline = addnewspline new_splineshape
	addknot new_splineshape new_spline #corner #line pos1

-- Loop through vertex locations

	for posnew = 2 to n do
	addknot new_splineshape new_spline #corner #line Sarrg [posnew]
	updateshape new_splineshape
	
	EndTimer()
	)
	
	
------------------------------------------------------------------------
SVerts()
)