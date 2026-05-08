bl_info = {
    "name": "Graph Closest Path",
    "author": "Abderraouf Benoudina",
    "version": (1, 2, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Addons",
    "description": "Find closest path between selected vertices",
    "category": "Mesh",
}

import bpy
import bmesh
import heapq
from bpy.props import EnumProperty


def build_graph(bm):
    edges = []
    edge_map = {}
    for e in bm.edges:
        v1 = e.verts[0].index
        v2 = e.verts[1].index
        w = (e.verts[0].co - e.verts[1].co).length
        edges.append((v1, v2, w))
        edges.append((v2, v1, w))
        edge_map[(min(v1, v2), max(v1, v2))] = e
    return edges, edge_map


def dijkstra(n, edges, start, blocked):
    graph = [[] for _ in range(n)]
    for u, v, w in edges:
        if u in blocked or v in blocked:
            continue
        graph[u].append((v, w))
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0.0
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if d != dist[u]:
            continue
        for v, w in graph[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, prev


def bellman_ford(n, edges, start, blocked):
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0.0
    for _ in range(n - 1):
        updated = False
        for u, v, w in edges:
            if u in blocked or v in blocked:
                continue
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                prev[v] = u
                updated = True
        if not updated:
            break
    return dist, prev


def bellman_kalaba(n, edges, start, blocked):
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0.0
    dist_old = [float("inf")] * n
    i = 0
    while dist_old != dist and i <= n:
        dist_old = dist[:]
        i += 1
        for u, v, w in edges:
            if u in blocked or v in blocked:
                continue
            if dist_old[u] + w < dist[v]:
                dist[v] = dist_old[u] + w
                prev[v] = u
    return dist, prev


def floyd_warshall(n, edges, blocked):
    dist = [[float("inf")] * n for _ in range(n)]
    nxt = [[None] * n for _ in range(n)]
    for i in range(n):
        if i in blocked:
            continue
        dist[i][i] = 0.0
        nxt[i][i] = i
    for u, v, w in edges:
        if u in blocked or v in blocked:
            continue
        if w < dist[u][v]:
            dist[u][v] = w
            nxt[u][v] = v
    for k in range(n):
        if k in blocked:
            continue
        for i in range(n):
            if i in blocked:
                continue
            if dist[i][k] == float("inf"):
                continue
            for j in range(n):
                if j in blocked:
                    continue
                nd = dist[i][k] + dist[k][j]
                if nd < dist[i][j]:
                    dist[i][j] = nd
                    nxt[i][j] = nxt[i][k]
    return dist, nxt


def reconstruct_path_prev(prev, start, end):
    path = []
    v = end
    while v is not None and v != start:
        path.append(v)
        v = prev[v]
    if v != start:
        return []
    path.append(start)
    path.reverse()
    return path


def reconstruct_path_floyd(nxt, start, end):
    if nxt[start][end] is None:
        return []
    path = [start]
    v = start
    while v != end:
        v = nxt[v][end]
        if v is None:
            return []
        path.append(v)
    return path


class MESH_OT_graph_closest_path(bpy.types.Operator):
    bl_idname = "mesh.graph_closest_path"
    bl_label = "Graph Closest Path"
    bl_options = {'REGISTER', 'UNDO'}

    method: EnumProperty(
        name="Method",
        items=[
            ("DIJKSTRA", "Dijkstra", "Shortest path by edge length"),
            ("BELLMAN_FORD", "Bellman-Ford", "Shortest path by relaxation"),
            ("BELLMAN_KALABA", "Bellman-Kalaba", "Iterative relaxation"),
            ("FLOYD", "Floyd", "All-pairs shortest paths"),
        ],
        default="DIJKSTRA",
    )

    _timer = None
    _anim_verts = []
    _anim_index = 0
    _final_edges = []
    _bm = None
    _obj = None

    def execute(self, context):
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a mesh in Edit Mode.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        sel_verts = [v for v in bm.verts if v.select]
        if len(sel_verts) < 2:
            self.report({'ERROR'}, "Select at least 2 vertices.")
            return {'CANCELLED'}

        active = bm.select_history.active
        if not active or not isinstance(active, bmesh.types.BMVert):
            self.report({'ERROR'}, "Active element must be a vertex.")
            return {'CANCELLED'}

        n = len(bm.verts)
        edges, edge_map = build_graph(bm)

        remaining = set(v.index for v in sel_verts)
        current = active.index
        origin_active = active.index

        if current not in remaining:
            remaining.add(current)

        first_step = True
        path_vertices_all = []
        path_edges_all = []

        while True:
            targets = [i for i in remaining if i != current]
            if not targets:
                break

            blocked = set()
            if not first_step:
                blocked.add(origin_active)

            if self.method == "DIJKSTRA":
                dist, prev = dijkstra(n, edges, current, blocked)
            elif self.method == "BELLMAN_FORD":
                dist, prev = bellman_ford(n, edges, current, blocked)
            elif self.method == "BELLMAN_KALABA":
                dist, prev = bellman_kalaba(n, edges, current, blocked)
            else:
                dist_matrix, nxt = floyd_warshall(n, edges, blocked)

            best = None
            best_dist = float("inf")
            for t in targets:
                d = dist_matrix[current][t] if self.method == "FLOYD" else dist[t]
                if d < best_dist:
                    best_dist = d
                    best = t

            if best is None or best_dist == float("inf"):
                break

            if self.method == "FLOYD":
                path = reconstruct_path_floyd(nxt, current, best)
            else:
                path = reconstruct_path_prev(prev, current, best)

            if not path:
                break

            for a, b in zip(path[:-1], path[1:]):
                if not first_step and b == origin_active:
                    break
                key = (min(a, b), max(a, b))
                e = edge_map.get(key)
                if e:
                    path_edges_all.append(e)

            if not path_vertices_all:
                path_vertices_all.extend(path)
            else:
                path_vertices_all.extend(path[1:])

            remaining.discard(best)
            current = best
            first_step = False

        self._anim_verts = path_vertices_all
        self._anim_index = 0
        self._final_edges = path_edges_all
        self._bm = bm
        self._obj = obj

        for v in bm.verts:
            v.select = False
        for e in bm.edges:
            e.select = False

        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type == 'TIMER':
            bm = self._bm
            if self._anim_index < len(self._anim_verts):
                idx = self._anim_verts[self._anim_index]
                if idx is not None and idx < len(bm.verts):
                    bm.verts[idx].select = True
                self._anim_index += 1
                bmesh.update_edit_mesh(self._obj.data, loop_triangles=False, destructive=False)
                return {'RUNNING_MODAL'}
            for v in bm.verts:
                v.select = False
            for e in bm.edges:
                e.select = False
            for e in self._final_edges:
                e.select = True
                e.verts[0].select = True
                e.verts[1].select = True
            bmesh.update_edit_mesh(self._obj.data, loop_triangles=False, destructive=False)
            context.window_manager.event_timer_remove(self._timer)
            return {'FINISHED'}
        return {'RUNNING_MODAL'}


class VIEW3D_PT_graph_closest_path(bpy.types.Panel):
    bl_label = "Graph Closest Path"
    bl_idname = "VIEW3D_PT_graph_closest_path"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Addons"

    def draw(self, context):
        layout = self.layout
        layout.prop(context.scene, "graph_cp_method")
        op = layout.operator(MESH_OT_graph_closest_path.bl_idname, text="Run")
        op.method = context.scene.graph_cp_method


def register():
    bpy.utils.register_class(MESH_OT_graph_closest_path)
    bpy.utils.register_class(VIEW3D_PT_graph_closest_path)
    bpy.types.Scene.graph_cp_method = EnumProperty(
        name="Method",
        items=[
            ("DIJKSTRA", "Dijkstra", "Shortest path by edge length"),
            ("BELLMAN_FORD", "Bellman-Ford", "Shortest path by relaxation"),
            ("BELLMAN_KALABA", "Bellman-Kalaba", "Iterative relaxation"),
            ("FLOYD", "Floyd", "All-pairs shortest paths"),
        ],
        default="DIJKSTRA",
    )


def unregister():
    del bpy.types.Scene.graph_cp_method
    bpy.utils.unregister_class(VIEW3D_PT_graph_closest_path)
    bpy.utils.unregister_class(MESH_OT_graph_closest_path)


if __name__ == "__main__":
    register()