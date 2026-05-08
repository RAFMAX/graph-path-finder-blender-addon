# Graph Closest Path (Blender Addon)

A Blender addon that connects selected mesh vertices by building paths segment by segment from the active vertex.  
It animates propagation and then selects the final combined path edges in **View3D Sidebar > Addons**.

## Features

- Works in **Edit Mode** on mesh objects
- Uses **selected vertices** and the **active vertex** as the start
- Repeats: current -> nearest selected vertex (path segment)
- Propagation animation per segment:
  - Start spreading from current vertex
  - Stop spreading as soon as a selected target is reached
  - Continue from that reached target to the next one
- Selects the final combined **path edges** in the mesh
- Algorithms:
  - Dijkstra
  - A*
  - BFS
  - DFS

## Install

1. Save the addon as `graph_closest_path_addon.py`
2. In Blender: **Edit > Preferences > Add-ons > Install...**
3. Select the file and enable it

## Usage

1. Enter **Edit Mode** on a mesh
2. Select **2 or more vertices**
3. Make one vertex **active** (last selected)
4. Open the **N‑panel** → **Addons** tab → **Graph Closest Path**
5. Choose the algorithm and click **Run**

## Notes

- After the first segment, the original active vertex is **blocked** to avoid passing through it again.
- The addon uses edge lengths as weights (Euclidean distance).
- Dijkstra and A* are weighted by edge length.
- BFS and DFS are unweighted traversal-based options.
