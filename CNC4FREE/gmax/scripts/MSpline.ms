fn find_intersection target_mesh s = 
( 
	local testRay = ray (getKnotPoint $ 1 s) [0,0,-1]
	local nodeMaxZ =  target_mesh.max.z 
	testRay.pos.z = nodeMaxZ + 0.0001 * abs nodeMaxZ 
	intersectRay  target_mesh testRay 
) 

target_mesh = pickObject message:"Pick Target Surface:"

if target_mesh != undefined do
		(
			numKnots $
			for s = 1 to  (numKnots $) do
				( 
				setKnotSelection $ 1 #(s)
				int_point = find_intersection target_mesh s
				if int_point != undefined then
				setKnotPoint $ 1 s int_point.pos 
				)--end s loop 
			)--end if

		updateShape $
		$.pivot = $.center

