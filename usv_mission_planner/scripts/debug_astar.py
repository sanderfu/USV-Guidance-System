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
    map_name = "outside_new_york"
    mission_name = "testmission"
    datasource_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/check_db.sqlite"
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

    #tile_size = 0.0005/2
    #colors = plt.cm.get_cmap("viridis",len(np.unique(z_normalized.round(decimals=4)))*2)
    #i=0
    #for index,row in tqdm(distance_df.iterrows(), total=distance_df.shape[0]):
    #    x,y = row["x_center"], row["y_center"]
    #    coords = np.array([[x-tile_size,y-tile_size],[x-tile_size,y+tile_size],[x+tile_size,y+tile_size],[x+tile_size,y-tile_size]])
    #    poly:Polygon = Polygon(coords)
    #    patch1 = PolygonPatch(poly, fc=mpl.colors.rgb2hex(colors(z_normalized[index])),ec=mpl.colors.rgb2hex(colors(z_normalized[index])), alpha=1, zorder=1)
    #    ax.add_patch(patch1)
    #    i+=1
    #    if(i>10000):
    #        break

    #print("Plot path")
    #Plot path
    path_path = rospack.get_path('usv_mission_planner')+"/data/debug/astar/path.csv"
    path_df = pd.read_csv(path_path)
    ax.plot(path_df["lon"],path_df["lat"],color="red",zorder=3,label="path")

    #print("Came from line segments")
    #Plot came-from line segments
    #came_from_path = rospack.get_path('usv_mission_planner')+"/data/debug/astar/came_from.csv"
    #came_from_df = pd.read_csv(came_from_path)
    #lines = []
    #for index,row in came_from_df.iterrows():
    #    line = [(row["lon_from"],row["lat_from"]),(row["lon_to"],row["lat_to"])]
    #    lines.append(line)
    #lc = mc.LineCollection(lines, linewidths=0.1,zorder=3)
    #ax.add_collection(lc)

    #Plot closed
    closed_path = rospack.get_path('usv_mission_planner')+"/data/debug/astar/closed.csv"
    closed_df = pd.read_csv(closed_path)
    ax.scatter(closed_df["lon"],closed_df["lat"],color="grey",zorder=3,label="closed")

    #Plot unexplored
    frontier_path = rospack.get_path('usv_mission_planner')+"/data/debug/astar/frontier.csv"
    frontier_df = pd.read_csv(frontier_path)
    ax.scatter(frontier_df["lon"],frontier_df["lat"],color="blue",zorder=3,label="frontier")


    #Plot quadtree
    quadtree_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/quadtree.csv"
    quadtree_df = pd.read_csv(quadtree_path)
    lines = []
    for index,row in quadtree_df.iterrows():
        line = [(row["u_lon"],row["u_lat"]),(row["v_lon"],row["v_lat"])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=0.1,zorder=3)
    ax.add_collection(lc)
    ax.legend()


    plt.autoscale(enable=True, axis="both", tight=None)
    plt.show()

main()
