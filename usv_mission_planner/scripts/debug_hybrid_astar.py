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
    datasource_path = rospack.get_path('usv_simulator')+"/maps/check_db.sqlite"
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
            patch1 = PolygonPatch(poly, fc=GREEN, ec=GREEN, alpha=0.5, zorder=2)
            ax.add_patch(patch1)

    #Plot exploration Hybrid A*
    ax.scatter(-73.972908,40.523693,color="r")
    ax.scatter(-73.974206,40.542943,color="g")
    exporation_path = rospack.get_path('usv_mission_planner')+"/data/debug/hybrid_astar/debug.csv"
    exporation_df = pd.read_csv(exporation_path)
    ax.scatter(exporation_df["lon"],exporation_df["lat"])
    for i in range(0,len(exporation_df["lon"])):
        ax.annotate(exporation_df["id"][i],(exporation_df["lon"][i],exporation_df["lat"][i]))

    plt.autoscale(enable=True, axis="both", tight=None)
    plt.show()


main()