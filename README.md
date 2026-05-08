# Graph Closest Path (Blender Addon)

A Blender addon that finds **shortest paths by edge length** on a mesh and **selects the path edges** between selected vertices.  
It supports multiple shortest‑path algorithms and runs from the **View3D Sidebar > Addons** panel.

## Features

- Works in **Edit Mode** on mesh objects
- Uses **selected vertices** and the **active vertex** as the start
- Repeats: current → nearest selected vertex (shortest path)
- Selects **path edges** in the mesh
- Algorithms:
  - Dijkstra
  - Bellman‑Ford
  - Bellman‑Kalaba
  - Floyd‑Warshall

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

- After the first segment, the original active vertex is **blocked** to avoid paths passing through it again.
- The addon uses edge lengths as weights (Euclidean distance).
- Floyd‑Warshall is heavier for large meshes.
