bl_info = {
    "name": "Graph Closest Path",
    "author": "Abderraouf Benoudina",
    "version": (1, 4, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Addons",
    "description": "Find closest path between selected vertices",
    "category": "Mesh",
}

import bpy
import bmesh
import heapq
from bpy.props import EnumProperty
from mathutils import Vector


def build_graph(bm):
    edges = []
    edge_map = {}
    coords = {}
    for v in bm.verts:
        coords[v.index] = v.co.copy()
    for e in bm.edges:
        v1 = e.verts[0].index
        v2 = e.verts[1].index
        w = (e.verts[0].co - e.verts[1].co).length
        edges.append((v1, v2, w))
        edges.append((v2, v1, w))
        edge_map[(min(v1, v2), max(v1, v2))] = e
    return edges, edge_map, coords


def build_adj(n, edges, blocked):
    graph = [[] for _ in range(n)]
    for u, v, w in edges:
        if u in blocked or v in blocked:
            continue
        graph[u].append((v, w))
    return graph


def levels_from_prev(prev, start):
    cache = {}
    def hops(v):
        if v == start:
            return 0
        if v in cache:
            return cache[v]
        p = prev[v]
        if p is None:
            return None
        h = hops(p)
        if h is None:
            return None
        cache[v] = h + 1
        return cache[v]
    levels = {}
    for v in range(len(prev)):
        h = hops(v)
        if h is not None:
            levels.setdefault(h, []).append(v)
    return [levels[k] for k in sorted(levels.keys())]


def levels_until_target(prev, start, target):
    cache = {}

    def hops(v):
        if v == start:
            return 0
        if v in cache:
            return cache[v]
        p = prev[v]
        if p is None:
            return None
        h = hops(p)
        if h is None:
            return None
        cache[v] = h + 1
        return cache[v]

    target_hops = hops(target)
    if target_hops is None:
        return []

    levels = {}
    for v in range(len(prev)):
        h = hops(v)
        if h is not None and h <= target_hops:
            levels.setdefault(h, []).append(v)
    return [levels[k] for k in sorted(levels.keys())]


def dijkstra_all(n, edges, start, blocked):
    graph = build_adj(n, edges, blocked)
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


def dijkstra_to_any_target(n, edges, start, targets, blocked):
    graph = build_adj(n, edges, blocked)
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0.0
    target_set = set(targets)
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if d != dist[u]:
            continue
        if u in target_set:
            return u, dist, prev
        for v, w in graph[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return None, dist, prev


def astar_to_target(n, edges, start, target, coords, blocked):
    graph = build_adj(n, edges, blocked)
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0.0
    def h(v):
        return (coords[v] - coords[target]).length
    pq = [(h(start), start)]
    visited = set()
    while pq:
        _, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if u == target:
            break
        for v, w in graph[u]:
            nd = dist[u] + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd + h(v), v))
    return dist, prev


def bfs_all(n, edges, start, blocked):
    graph = build_adj(n, edges, blocked)
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0
    q = [start]
    head = 0
    while head < len(q):
        u = q[head]
        head += 1
        for v, _ in graph[u]:
            if dist[v] == float("inf"):
                dist[v] = dist[u] + 1
                prev[v] = u
                q.append(v)
    return dist, prev


def bfs_to_any_target(n, edges, start, targets, blocked):
    graph = build_adj(n, edges, blocked)
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0
    target_set = set(targets)
    q = [start]
    head = 0
    while head < len(q):
        u = q[head]
        head += 1
        if u in target_set:
            return u, dist, prev
        for v, _ in graph[u]:
            if dist[v] == float("inf"):
                dist[v] = dist[u] + 1
                prev[v] = u
                q.append(v)
    return None, dist, prev


def dfs_all(n, edges, start, blocked):
    graph = build_adj(n, edges, blocked)
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0
    stack = [start]
    visited = set()
    while stack:
        u = stack.pop()
        if u in visited:
            continue
        visited.add(u)
        for v, _ in graph[u]:
            if v not in visited and dist[v] == float("inf"):
                dist[v] = dist[u] + 1
                prev[v] = u
                stack.append(v)
    return dist, prev


def dfs_to_any_target(n, edges, start, targets, blocked):
    graph = build_adj(n, edges, blocked)
    dist = [float("inf")] * n
    prev = [None] * n
    dist[start] = 0
    target_set = set(targets)
    stack = [start]
    visited = set()
    while stack:
        u = stack.pop()
        if u in visited:
            continue
        visited.add(u)
        if u in target_set:
            return u, dist, prev
        for v, _ in graph[u]:
            if v not in visited and dist[v] == float("inf"):
                dist[v] = dist[u] + 1
                prev[v] = u
                stack.append(v)
    return None, dist, prev


def reconstruct_path(prev, start, end):
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


class MESH_OT_graph_closest_path(bpy.types.Operator):
    bl_idname = "mesh.graph_closest_path"
    bl_label = "Graph Closest Path"
    bl_options = {'REGISTER', 'UNDO'}

    method: EnumProperty(
        name="Method",
        items=[
            ("DIJKSTRA", "Dijkstra", "Shortest path by edge length"),
            ("ASTAR", "A*", "Heuristic shortest path"),
            ("BFS", "BFS", "Shortest by hop count"),
            ("DFS", "DFS", "Depth-first search"),
        ],
        default="DIJKSTRA",
    )

    _timer = None
    _steps = []
    _step_index = 0
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
        edges, edge_map, coords = build_graph(bm)

        remaining = set(v.index for v in sel_verts)
        current = active.index
        origin_active = active.index

        if current not in remaining:
            remaining.add(current)

        first_step = True
        path_edges_all = []
        steps_all = []

        while True:
            targets = [i for i in remaining if i != current]
            if not targets:
                break

            blocked = set()
            if not first_step:
                blocked.add(origin_active)

            if self.method == "DIJKSTRA":
                best, _, prev = dijkstra_to_any_target(n, edges, current, targets, blocked)
            elif self.method == "BFS":
                best, _, prev = bfs_to_any_target(n, edges, current, targets, blocked)
            elif self.method == "DFS":
                best, _, prev = dfs_to_any_target(n, edges, current, targets, blocked)
            else:
                # Pick the nearest target first, then build a precise segment with A*.
                best, _, _ = dijkstra_to_any_target(n, edges, current, targets, blocked)
                prev = None

            if best is None:
                break

            if self.method == "ASTAR":
                dist_a, prev = astar_to_target(n, edges, current, best, coords, blocked)

            path = reconstruct_path(prev, current, best)
            if not path:
                break

            # Show propagation, but stop the animation once the reached target appears.
            levels = levels_until_target(prev, current, best)
            steps_all.append(None)
            steps_all.extend(levels)

            for a, b in zip(path[:-1], path[1:]):
                if not first_step and b == origin_active:
                    break
                key = (min(a, b), max(a, b))
                e = edge_map.get(key)
                if e:
                    path_edges_all.append(e)

            remaining.discard(best)
            current = best
            first_step = False

        self._steps = steps_all
        self._step_index = 0
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
            if self._step_index < len(self._steps):
                step = self._steps[self._step_index]
                if step is None:
                    for v in bm.verts:
                        v.select = False
                    for e in bm.edges:
                        e.select = False
                else:
                    for idx in step:
                        if idx is not None and idx < len(bm.verts):
                            bm.verts[idx].select = True
                self._step_index += 1
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
            ("ASTAR", "A*", "Heuristic shortest path"),
            ("BFS", "BFS", "Shortest by hop count"),
            ("DFS", "DFS", "Depth-first search"),
        ],
        default="DIJKSTRA",
    )


def unregister():
    del bpy.types.Scene.graph_cp_method
    bpy
