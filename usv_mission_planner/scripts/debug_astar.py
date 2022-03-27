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
            "axes.grid": False,
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
    mission_name = "test_adaptive_TRD_ON"
    datasource_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/region.sqlite"
    ds:gdal.Dataset = gdal.OpenEx(datasource_path)
    if ds==None:
        raise RuntimeError("Failed to load datasource",datasource_path)
    collision_layer:ogr.Layer = ds.GetLayerByName("collision_dissolved")

    figure,ax = plt.subplots(1,1)
    figure_2, ax_2 = plt.subplots(2,1)
    #Plot benchmark
    early_exit_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/benchmark/early_exit.csv"
    early_exit_df = pd.read_csv(early_exit_path)
    sequence_match_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/benchmark/sequence_match.csv"
    sequence_match_df = pd.read_csv(sequence_match_path)
    ax_2[0].set_title("Early exit search times")
    ax_2[0].set_ylabel("time[s]")
    ax_2[0].plot(early_exit_df["time"])

    ax_2[1].set_title("Sequence match search times")
    ax_2[1].set_ylabel("time[s]")
    ax_2[1].plot(sequence_match_df["time"])


    #Plot background
    collision_layer.ResetReading()
    for feat in collision_layer:
        for geom in feat.GetGeometryRef():
            wkt = geom.ExportToWkt()
            poly:Polygon = Polygon(loads(wkt))
            patch1 = PolygonPatch(poly, fc=GREEN, ec=GREEN, alpha=1, zorder=2)
            ax.add_patch(patch1)
    
    #Plot quadtree
    #quadtree_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/quadtree.csv"
    #quadtree_df = pd.read_csv(quadtree_path)
    #lines = []
    #for index,row in quadtree_df.iterrows():
    #    line = [(row["u_lon"],row["u_lat"]),(row["v_lon"],row["v_lat"])]
    #    lines.append(line)
    #lc = mc.LineCollection(lines, linewidths=0.1,zorder=3)
    #ax.add_collection(lc)

    a_star_search_count = (len(next(os.walk(rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+"/astar/search/"))[1]))

    for i in range(0,a_star_search_count):
        #Plot path and start
        path_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/search/{i}/path.csv"
        print(path_path)
        path_df = pd.read_csv(path_path)
        path_plot = ax.plot(path_df["lon"],path_df["lat"],color="red",zorder=3)
        start_scatter = ax.scatter(path_df["lon"][0],path_df["lat"][0],color="black",marker="*",zorder=4)
        ax.annotate(str(i), (path_df["lon"][0], path_df["lat"][0]),fontsize=10)

        #Plot closed
        #closed_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/search/{i}/closed.csv"
        #closed_df = pd.read_csv(closed_path)
        #closed_scatter = ax.scatter(closed_df["lon"],closed_df["lat"],color="grey",zorder=3,label="closed")

        #Plot unexplored
        #frontier_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/search/{i}/frontier.csv"
        #frontier_df = pd.read_csv(frontier_path)
        #frontier_scatter = ax.scatter(frontier_df["lon"],frontier_df["lat"],color="blue",zorder=3,label="frontier")

        #Plot sequence match
        sequence_match_path = rospack.get_path('usv_mission_planner')+"/data/missions/"+mission_name+f"/astar/search/{i}/match_sequence.csv"
        sequence_match_df = pd.read_csv(sequence_match_path)
        sequence_match_plot = ax.plot(sequence_match_df["lon"],sequence_match_df["lat"],color="orange",zorder=3)
        ax.scatter(sequence_match_df["lon"],sequence_match_df["lat"],color="orange",marker="*",zorder=4)

        #path_plot.pop(0).remove()
        #sequence_match_plot.pop(0).remove()
        #start_scatter.remove()
        

        #ax.lines.pop(0)


    ax.legend()
    plt.show()


    #plt.autoscale(enable=True, axis="both", tight=None)
    #plt.show()

main()
