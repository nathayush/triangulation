import open3d as o3d
import trimesh
import numpy as np

print("reading input pointcloud")
pcd = o3d.io.read_point_cloud("pointclouds/beethoven.ply")
print("estimating normals")
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=6))

# estimate radius for rolling ball
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 1.5 * avg_dist   

print("calculating mesh")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,
           o3d.utility.DoubleVector([radius, radius * 2]))

# create the triangular mesh with the vertices and faces from open3d
tri_mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles),
                          vertex_normals=np.asarray(mesh.vertex_normals))

print("exporting mesh")
trimesh.convex.is_convex(tri_mesh)
tri_mesh.export("beethoven.stl")
print("done")
