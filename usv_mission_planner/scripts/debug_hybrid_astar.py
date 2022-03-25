import matplotlib.pyplot as plt
import matplotlib.collections  as mc
import pandas as pd
import numpy as np
from tqdm import tqdm
from osgeo import ogr, osr, gdal
from descartes import PolygonPatch
from shapely.geometry import Polygon
from shapely.wkt import loads
import rospkg
import matplotlib as mpl

try:
    # installed with "pip install SciencePLots" (https://github.com/garrettj403/SciencePlots.git)
    # gives quite nice plots
    plt_styles = ["science", "grid", "bright", "no-latex"]
    plt.style.use(plt_styles)
    print(f"pyplot using style set {plt_styles}")
except Exception as e:
    plt.rcParams.update(
        {
            # setgrid
            "axes.grid": True,
            "grid.linestyle": ":",
            "grid.color": "k",
            "grid.alpha": 0.5,
            "grid.linewidth": 0.1,
            # Legend
            "legend.frameon": True,
            "legend.framealpha": 1.0,
            "legend.fancybox": True,
            "legend.numpoints": 1,
            "legend.loc" : "upper right",
            'legend.fontsize': 15,
            # Font
            "font.size" : 15,
            #Subplots and figure
            "figure.figsize" : [8,7],
            "figure.subplot.wspace" : 0.37,
            "figure.subplot.hspace" : 0.76,
            "figure.subplot.top" : 0.9,
            "figure.subplot.right" : 0.95,
            "figure.subplot.left" : 0.1,
        }
    )

BLUE = '#6699cc'
GRAY = '#999999'
GREEN = '#4F7942'

def main():
    rospack = rospkg.RosPack()
    map_name = "trondheim_hitra_4x"
    mission_name = "test_matchsequence_trondheim_4x_105_360"
    datasource_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/region.sqlite"
    ds:gdal.Dataset = gdal.OpenEx(datasource_path)
    if ds==None:
        raise RuntimeError("Failed to load datasource",datasource_path)
    collision_layer:ogr.Layer = ds.GetLayerByName("collision_dissolved")

    figure,ax = plt.subplots(1,1)

    #Plot background
    collision_layer.ResetReading()
    for feat in collision_layer:
        for geom in feat.GetGeometryRef():
            wkt = geom.ExportToWkt()
            poly:Polygon = Polygon(loads(wkt))
            patch1 = PolygonPatch(poly, fc=GREEN, ec=GREEN, alpha=1, zorder=2)
            ax.add_patch(patch1)

    #Plot distance coloured
    #distance_path = rospack.get_path('usv_map')+"/data/debug_distance_concept/distance_tiles.csv"
    #distance_df = pd.read_csv(distance_path)
    #z_normalized = (z-min(z))/(max(z)-min(z))

    #tile_size = 0.001/2
    #colors = plt.cm.get_cmap("Greys",len(np.unique(z_normalized.round(decimals=4)))*2)
    #i=0
    #for index,row in tqdm(distance_df.iterrows(), total=distance_df.shape[0]):
    #    x,y = row["x_center"], row["y_center"]
    #    coords = np.array([[x-tile_size,y-tile_size],[x-tile_size,y+tile_size],[x+tile_size,y+tile_size],[x+tile_size,y-tile_size]])
    #    poly:Polygon = Polygon(coords)
    #    patch1 = PolygonPatch(poly, fc=mpl.colors.rgb2hex(colors(z_normalized[index])),ec=mpl.colors.rgb2hex(colors(z_normalized[index])), alpha=1, zorder=1)
    #    ax.add_patch(patch1)
    #    i+=1

    #Plot path
    path_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/hybrid_astar/path.csv"
    path_df = pd.read_csv(path_path)
    ax.plot(path_df["lon"],path_df["lat"],color="red",zorder=3,label="path")

    #Plot came-from line segments
    came_from_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/hybrid_astar/came_from.csv"
    came_from_df = pd.read_csv(came_from_path)
    lines = []
    for index,row in came_from_df.iterrows():
        line = [(row["lon_from"],row["lat_from"]),(row["lon_to"],row["lat_to"])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=0.1,zorder=3)
    ax.add_collection(lc)

    #Plot closed
    plotpause = 0.00001
    closed_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/hybrid_astar/closed.csv"
    closed_df = pd.read_csv(closed_path)
    #for index, row in closed_df.iterrows():
    #    point = ax.scatter(row["lon"],row["lat"],color="grey",zorder=3)
    #    figure.canvas.draw_idle()
    #    plt.show(block=False)
    #    plt.waitforbuttonpress(plotpause)
    #    point.remove()

    ax.scatter(closed_df["lon"],closed_df["lat"],color="grey",zorder=3,label="closed")

    #Plot unexplored
    frontier_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/hybrid_astar/frontier.csv"
    frontier_df = pd.read_csv(frontier_path)
    ax.scatter(frontier_df["lon"],frontier_df["lat"],color="blue",zorder=3,label="frontier")

    #Plot vertices outside quadtree
    outside_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/hybrid_astar/outside_quadtree.csv"
    outside_df = pd.read_csv(outside_path)
    ax.scatter(outside_df["lon"],outside_df["lat"],color="red",marker="x",zorder=3,label="fastcheck")

    #Plot candidates
    candidates_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/hybrid_astar/candidate_exploration.csv"
    candidates_df = pd.read_csv(candidates_path)
    #for index, row in candidates_df.iterrows():
    #    ax.scatter(row["cand_lon"],row["cand_lat"],zorder=3,color="pink")
    #    ax.annotate(row["cand_h"], (row["cand_lon"], row["cand_lat"]))


    #Plot quadtree
    #quadtree_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/quadtree.csv"
    #quadtree_df = pd.read_csv(quadtree_path)
    #lines = []
    #for index,row in quadtree_df.iterrows():
    #    line = [(row["u_lon"],row["u_lat"]),(row["v_lon"],row["v_lat"])]
    #    lines.append(line)
    #lc = mc.LineCollection(lines, linewidths=0.1,zorder=3)
    #ax.add_collection(lc)
    #ax.legend()


    plt.autoscale(enable=True, axis="both", tight=None)
    plt.show()



main()