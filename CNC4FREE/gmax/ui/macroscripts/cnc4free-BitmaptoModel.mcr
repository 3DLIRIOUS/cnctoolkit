macroScript Img2Model
	category:"cnc4free"
	

(
	fn makeBPlane=(
		
		BPlane = Plane()
		BPlane.name ="DisplaceMesh"
		BPlane.length = 100
		BPlane.width = 100
		BPlane.lengthsegs = 500
		BPlane.widthsegs = 500
		BPlane.wirecolor = color 214 229 166
		BPlane.mapCoords = on
		Deformit = displace()
		Deformit.maptype = 0
		Deformit.blur = 0.00
		Deformit.width = 0.9
		Deformit.length = 0.9
		Deformit.strength = 3.0
		addmodifier BPlane Deformit
		BPlane.isSelected = on
		)		

ClearListener()
makeBPlane()
Max Modify Mode
)