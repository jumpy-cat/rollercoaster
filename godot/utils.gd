extends Node

## Adds 3d lines between [points] with the given [radius]
static func cylinder_line(m, points, radius):
	if len(points) > 1:
		for i in range(1, len(points)):
			# create cylinder
			const NUM_QUADS = 10
			var p = points[i-1]
			var q = points[i]
			var pq: Vector3 = q - p
			var rot = Quaternion(pq.normalized(), Vector3.UP).normalized()
			
			for j in range(NUM_QUADS):
				var theta0 = 2*PI * j / NUM_QUADS
				var theta1 = 2*PI * (j+1) / NUM_QUADS
				var offset0 = (Vector3(1, 0, 0) * Quaternion(Vector3.UP, theta0)) * rot
				var offset1 = (Vector3(1, 0, 0) * Quaternion(Vector3.UP, theta1)) * rot
				var quad = [
					p + radius * (offset1),
					q + radius * (offset1),
					q + radius * (offset0),
					p + radius * (offset0),
					
					offset1,
					offset1,
					offset0,
					offset0,
				]
				add_quad(
					m,
					quad[0], quad[1], quad[2], quad[3],
					quad[4], quad[5], quad[6], quad[7]
				)


## Adds a quad to ImmediateMesh
## CCW winding order
static func add_quad(m, p1, p2, p3, p4, n1, n2, n3, n4):
	add_tri(m, p1, p2, p3, n1, n2, n3)
	add_tri(m, p3, p4, p1, n3, n4, n1)

## Adds a triangle to ImmediateMesh
## CCW winding order
static func add_tri(m, p1, p2, p3, n1, n2, n3):
	m.surface_set_normal(n1)
	m.surface_add_vertex(p1)
	m.surface_set_normal(n2)
	m.surface_add_vertex(p2)
	m.surface_set_normal(n3)
	m.surface_add_vertex(p3)

