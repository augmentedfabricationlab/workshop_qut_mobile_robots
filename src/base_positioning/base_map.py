from compas.datastructures import Network
from compas.geometry import Point, Pointcloud
from compas.geometry import Frame, Vector, Transformation, Translation
from compas.geometry import KDTree, closest_point_in_cloud, closest_points_in_cloud_numpy
from numpy import meshgrid, linspace
from statistics import mean, median_low, median_high
import math

class BaseMap(object):
    def __init__(self):
        self.base_map = Network(name="base_map")
        self.base_map.default_node_attributes = {
            "frame": Frame.worldXY(),
            "point": [0, 0, 0],
            "xaxis": [1, 0, 0],
            "yaxis": [0, 1, 0],
            "valid_position": True,
            "mean_reuleax_index": None,
            "mean_reach_index": -1,
            "reuleaux_data": None,
            "closeness_penalty": 0,
        }

    @property
    def kdtree(self):
        objects = []
        for key, data in self.base_map.nodes(data=True):
            objects.append((data["frame"].point, key))
        kdtree = KDTree()
        kdtree.build(objects)
        return kdtree

    def add_pose(self, pose, **kwargs):
        return self.base_map.add_node(
            frame=pose, point=pose.point, xaxis=pose.xaxis, yaxis=pose.yaxis, **kwargs
        )

    def set_poses(self, points, attractor_point=[0, 0, 0]):
        self.base_map.clear()
        for i, pt in enumerate(points):
            vec = Vector.from_start_end(pt, attractor_point).unitized()
            self.base_map.add_node(
                key=i, x=pt.x, y=pt.y, z=pt.z, vx=vec.x, vy=vec.y, vz=vec.z
            )

    def reuleaux_at_pose(self, pose, reuleaux):
        T = Transformation.from_change_of_basis(pose, Frame.worldXY())
        kdtree, reuleaux_points = reuleaux.get_kdtree_transformed(T)
        # print("t2: ", time()-t0)
        reuleaux_data = {
            "goal_points": None,
            "collision_points": None,
            "reuleaux_points": reuleaux_points,
            "goal_reuleaux_index": [],
            "collision_reuleaux_index": [],
            "reachable": [],
        }
        return kdtree, reuleaux_data

    def evaluate_pose(self, kdtree, reuleaux, reuleaux_data, goal_pointcloud):
        ## TODO should add minimum required reachability index
        reuleaux_data["goal_points"] = goal_pointcloud.points
        goal_data = [
            kdtree.nearest_neighbors(pt, 2, distance_sort=True)
            for pt in goal_pointcloud.points
        ]

        for d in goal_data:
            eval_data = {"goal_reuleaux_index": 0, "reachable": False}
            # Check if two points of the reachability map are within a set distance (removes points close enough to the boundary of the map)
            if d[0][2] < (reuleaux.resolution * 1.415) and d[1][2] < (
                reuleaux.resolution * 1.415
            ):
                point_ri = math.sqrt(reuleaux.spheres[d[0][1]].ri)
                eval_data.update({"goal_reuleaux_index": point_ri, "reachable": True})
            for key, value in eval_data.items():
                reuleaux_data[key].append(value)

        ri_mean = median_low(reuleaux_data["goal_reuleaux_index"])
        return ri_mean, reuleaux_data

    def reuleaux_kdtree_includes_points(self, kdtree, resolution, points):
        return all(
            [
                kdtree.nearest_neighbor(pt)[2] <= resolution * (0.5 * 1.415)
                for pt in points
            ]
        )

    def get_reuleaux_index_by_base_pose(self, pose, reuleaux, goal_pointcloud):
        if reuleaux.resolution == 0:
            raise ValueError("ReuleauxReachability instance has resolution == 0.0")
        closest_map_point, id, distance = self.kdtree.nearest_neighbor(pose.point)
        if distance > 0.01:
            id = self.add_pose(pose)
        if self.base_map.node_attribute(id, "mean_reuleaux_index") is None:
            kdtree, reuleaux_data = self.reuleaux_at_pose(pose, reuleaux)
            ri_mean, reuleaux_data = self.evaluate_pose(
                kdtree, reuleaux, reuleaux_data, goal_pointcloud
            )

            self.base_map.node_attribute(id, "mean_reuleaux_index", ri_mean)
            self.base_map.node_attribute(id, "reuleaux_data", reuleaux_data)
        else:
            ri_mean = self.base_map.node_attribute(id, "mean_reuleaux_index")
            reuleaux_data = self.base_map.node_attribute(id, "reuleaux_data")
        return id, reuleaux_data

    def get_reuleaux_index_with_req_points(
        self, pose, reuleaux, goal_pointcloud, required_points
    ):
        if reuleaux.resolution == 0:
            raise ValueError("ReuleauxReachability instance has resolution == 0.0")
        closest_map_point, id, distance = self.kdtree.nearest_neighbor(pose.point)
        if distance > 0.01:
            id = self.add_pose(pose)
        if self.base_map.node_attribute(id, "mean_reuleaux_index") is None:
            kdtree, reuleaux_data = self.reuleaux_at_pose(pose, reuleaux)
            if self.reuleaux_kdtree_includes_points(
                kdtree, reuleaux.resolution, required_points.points
            ):
                ri_mean, reuleaux_data = self.evaluate_pose(
                    kdtree, reuleaux, reuleaux_data, goal_pointcloud
                )

                self.base_map.node_attribute(id, "mean_reuleaux_index", ri_mean)
                self.base_map.node_attribute(id, "reuleaux_data", reuleaux_data)
            else:
                self.base_map.node_attribute(id, "mean_reuleaux_index", 0)
                reuleaux_data["goal_reuleaux_index"] = [0]
                self.base_map.node_attribute(id, "reuleaux_data", reuleaux_data)
        else:
            ri_mean = self.base_map.node_attribute(id, "mean_reuleaux_index")
            reuleaux_data = self.base_map.node_attribute(id, "reuleaux_data")
        return id, reuleaux_data

    def get_ideal_pose(self):
        ideal_key = None
        mean_ri = None
        node_data = None
        for key, data in self.base_map.nodes(data=True):
            if mean_ri < data["mean_reuleaux_index"] or mean_ri is None:
                ideal_key = key
                mean_ri = data["mean_reuleaux_index"]
                node_data = data
        return ideal_key, node_data

    def get_ideal_pose_with_closeness_penalty(self, pointcloud, distance=1.0, factor=1):
        ideal_key = None
        mean_ri = None
        node_data = None
        for key, data in self.base_map.nodes(data=True):
            dist, pt, idx = closest_point_in_cloud(data["point"], pointcloud)
            if dist < distance:
                data["penalty_factor"] = 1 - (dist / distance) / factor
            else:
                data["penalty_factor"] = 0
            node_mean_ri = data["mean_reuleaux_index"] * (1 - data["penalty_factor"])
            if mean_ri is None or mean_ri < node_mean_ri:
                ideal_key = key
                mean_ri = node_mean_ri
                node_data = data
        return ideal_key, node_data

    def get_ideal_pose_with_closeness_penalty_and_mean_distance(self, pointcloud, distance=1.0, factor=1):
        ideal_key = None
        mean_dist = None
        mean_ri = None
        node_data = None
        mean_dists = []
        for key, data in self.base_map.nodes(data=True):
            kdtree = KDTree(pointcloud)
            result = kdtree.nearest_neighbors(data["point"], 20, distance_sort=True)
            pt, idx, dist = result[0]
            vec = Vector.from_start_end(data["point"], pt)
            vec.z = 0
            dist = vec.length
            if dist < distance:
                data["penalty_factor"] = math.sqrt(1 - (dist / distance) / factor)
            else:
                data["penalty_factor"] = 0
            node_mean_ri = data["mean_reuleaux_index"] * (1 - data["penalty_factor"])
            node_mean_dist = median_high([r[2] for r in result])
            mean_dists.append(node_mean_dist)
            if mean_ri is None or (mean_ri/mean_dist <= node_mean_ri/node_mean_dist):
                ideal_key = key
                mean_ri = node_mean_ri
                mean_dist = node_mean_dist
                node_data = data
        return ideal_key, node_data, mean_dists

    def get_ideal_pose_with_segment_and_collision_objects(self, segment_pointcloud, collision_breps, segment_distance=1.0, collision_distance=1.0, factor=1):
        ideal_key = None
        mean_dist = None
        mean_ri = None
        node_data = None
        mean_dists = []
        for key, data in self.base_map.nodes(data=True):
            kdtree = KDTree(segment_pointcloud)
            segment_result = kdtree.nearest_neighbors(data["point"], 20, distance_sort=True)
            pt, idx, dist = segment_result[0]
            vec = Vector.from_start_end(data["point"], pt)
            vec.z = 0
            dist = vec.length

            if dist < segment_distance:
                data["penalty_factor"] = math.sqrt(1 - (dist / segment_distance) / factor)
            else:
                data["penalty_factor"] = 0

            import ghpythonlib.components as ghcomp
            import Rhino.Geometry as rg

            brep_distance = None
            for cbrep in collision_breps:
                collision_result = ghcomp.BrepClosestPoint(rg.Point3d(*data["point"]), cbrep)
                # print(collision_result)
                if brep_distance is None:
                    brep_distance = collision_result["distance"]
                elif collision_result["distance"] < brep_distance:
                    brep_distance = collision_result["distance"]
            # print(brep_distance, collision_distance)
            if brep_distance < collision_distance:
                data["collision_penalty_factor"] = math.sqrt(1 - (brep_distance / collision_distance) / factor)
            else: 
                data["collision_penalty_factor"] = 0

            sum_penalties = data["penalty_factor"] + data["collision_penalty_factor"]
            if data["penalty_factor"] + data["collision_penalty_factor"] > 1:
                sum_penalties = 1.0

            node_mean_ri = data["mean_reuleaux_index"] * (1 - sum_penalties)
            node_mean_dist = median_high([r[2] for r in segment_result])
            # print(node_mean_ri, node_mean_dist)
            mean_dists.append(node_mean_dist)
            if mean_ri is None or (mean_ri/mean_dist <= node_mean_ri/node_mean_dist):
                ideal_key = key
                mean_ri = node_mean_ri
                mean_dist = node_mean_dist
                node_data = data
        return ideal_key, node_data
    
    def populate_rectangle(self, rectangle, resolution):
        w = rectangle.Width
        h = rectangle.Height
        frame = Frame(
            rectangle.Plane.Origin, rectangle.Plane.XAxis, rectangle.Plane.YAxis
        )

        nu = int(w / resolution)
        nv = int(h / resolution)

        u = linspace(-w / 2, w / 2, nu)
        v = linspace(h / 2, -h / 2, nv)

        X, Y = meshgrid(u, v)

        def flatten(matrix):
            flat_list = []
            for row in matrix:
                flat_list += list(row)
            return flat_list

        X_flat = flatten(X)
        Y_flat = flatten(Y)
        # pts_crds = zip(X, Y)
        print(X)
        pts = [Point(x, y, 0) for x, y in zip(X_flat, Y_flat)]
        print(pts)
        pc = Pointcloud(pts)
        T = Transformation.from_change_of_basis(frame, Frame.worldXY())
        pc.transform(T)

        return pc

    def populate_surface(self, surface, number_of_points):
        # points = populate(surface, number of points)
        # for i, pt in enumerate(points):
        #   node = {
        #       'x':pt.x, 'y':pt.y, 'z':pt.z,
        #   }
        #   self.base_map.add_node(key=i, attr_dict=node)
        pass

    def populate_around_point(self, surface, point, number_of_points, radius):
        # sphere(point, radius)
        # cut_surface with sphere
        # populate area with new points
        # for i, pt in enumerate(points):
        #   node = {
        #       'x':pt.x, 'y':pt.y, 'z':pt.z,
        #   }
        #   self.base_map.add_node(key=i, attr_dict=node)
        pass
