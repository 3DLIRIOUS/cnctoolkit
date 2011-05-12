macroScript ExportOBJ

category:"cnc4free"

------------------------------------------------------------------------------------------
(

ClearListener()
start = timeStamp()

global baseVertex = 0 
global baseTVertex = 0 
global baseNVertex = 0
global g_ModelName = Output

function toDec f = ( 
local i = f as integer 
return i 
) 

function outputHeader =( 
format "\n" 
format "cnc4free.org modified .OBJ export script\n"
format "Texture Vertex and Face Normal listing removed.\n"
format "Processing Timer added.  Grab this file with a\n" 
format "Scriptgrabber to save the .OBJ file.\n" 
format "\n" 
format "# -----------------\n" 
format "# Start of obj file\n" 
flushStringStream 
) 

function outputFooter =( 
format "# end of obj file\n" 
format "# ---------------\n" 
end = timeStamp() 
format "# Processing took % minutes\n" ((end - start) / 60000.0) 
format "\n" 
flushStringStream 
) 

function outputNode node = ( 
format "o %\n" node.name 

local m = snapshotAsMesh node 
for v = 1 to m.numVerts do ( 
local vert = m.verts[v] 
format "v % % %\n" vert.pos.x vert.pos.y vert.pos.z 
flushStringStream 
) 

local mv = m.verts 
for f =1 to m.numFaces do ( 
local vface =getface m f 
local x = baseVertex + toDec(vface.x) 
local y = baseVertex + toDec(vface.y) 
local z = baseVertex + toDec(vface.z) 


( 
format "f % % %\n" x y z 
) 
) 
format "\n" 
baseVertex = baseVertex + m.numVerts 
) 

function PrintObj = ( 
outputHeader() 
for c in rootNode.children do outputNode c 
outputFooter() 
) 

PrintObj()

end = timeStamp() 
)	