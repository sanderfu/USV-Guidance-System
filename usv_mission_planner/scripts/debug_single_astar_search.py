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
import os

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
    map_name = "outside_new_york_2"
    mission_name = "test_matchsequence"
    search_iteration=1
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
    
    #Plot quadtree
    quadtree_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/quadtree.csv"
    quadtree_df = pd.read_csv(quadtree_path)
    lines = []
    for index,row in quadtree_df.iterrows():
        line = [(row["u_lon"],row["u_lat"]),(row["v_lon"],row["v_lat"])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=0.1,zorder=3)
    ax.add_collection(lc)

    #Load path
    path_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/{search_iteration}/path.csv"
    print(path_path)
    path_df = pd.read_csv(path_path)

    #load closed
    closed_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/{search_iteration}/closed.csv"
    closed_df = pd.read_csv(closed_path)

    #Load frontier
    frontier_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/{search_iteration}/frontier.csv"
    frontier_df = pd.read_csv(frontier_path)

    #Load sequence match
    sequence_match_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/{search_iteration}/match_sequence.csv"
    sequence_match_df = pd.read_csv(sequence_match_path)


    

