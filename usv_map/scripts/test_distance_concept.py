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
import matplotlib.cm as cm
import matplotlib as mpl
from scipy.interpolate import griddata

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
DARK_GREEN = '#3d5e33'

def main():
    rospack = rospkg.RosPack()
    figure,ax = plt.subplots(1,1)
    figure_2,ax_2 = plt.subplots(1,1)
    figure_3,ax_3 = plt.subplots(1,1)
    #Plot background
    datasource_path = rospack.get_path('voroni')+"/data/test_map/check_db.sqlite"
    ds:gdal.Dataset = gdal.OpenEx(datasource_path)
    if ds==None:
        raise RuntimeError("Failed to load datasource",datasource_path)
    collision_layer:ogr.Layer = ds.GetLayerByName("collision_dissolved")

    #Plot background
    collision_layer.ResetReading()
    for feat in collision_layer:
        for geom in feat.GetGeometryRef():
            wkt = geom.ExportToWkt()
            poly:Polygon = Polygon(loads(wkt))
            poly2:Polygon = Polygon(loads(wkt))
            poly3:Polygon = Polygon(loads(wkt))
            patch1 = PolygonPatch(poly, fc=GREEN, ec=DARK_GREEN, alpha=1, zorder=2)
            ax.add_patch(patch1)
    
    #Plot background
    collision_layer.ResetReading()
    for feat in collision_layer:
        for geom in feat.GetGeometryRef():
            wkt = geom.ExportToWkt()
            poly:Polygon = Polygon(loads(wkt))
            poly2:Polygon = Polygon(loads(wkt))
            poly3:Polygon = Polygon(loads(wkt))
            patch1 = PolygonPatch(poly, fc=GREEN, ec=DARK_GREEN, alpha=1, zorder=2)
            ax_2.add_patch(patch1)

    #Plot background
    collision_layer.ResetReading()
    for feat in collision_layer:
        for geom in feat.GetGeometryRef():
            wkt = geom.ExportToWkt()
            poly:Polygon = Polygon(loads(wkt))
            poly2:Polygon = Polygon(loads(wkt))
            poly3:Polygon = Polygon(loads(wkt))
            patch1 = PolygonPatch(poly, fc=GREEN, ec=DARK_GREEN, alpha=1, zorder=2)
            ax_3.add_patch(patch1)
    
    


    #Plot distance coloured
    distance_path = rospack.get_path('usv_map')+"/data/debug_distance_concept/distance_tiles.csv"
    distance_df = pd.read_csv(distance_path)
    z = distance_df["distance_voronoi_field"]
    z_normalized = (z-min(z))/(max(z)-min(z))

    z_2 = -distance_df["distance_obstacle"]
    z_2_normalized = (z_2-min(z_2))/(max(z_2)-min(z_2))

    z_3 = distance_df["distance_voronoi"]
    z_3_normalized = (z_3-min(z_3))/(max(z_3)-min(z_3))

    tile_size = 0.001/2
    colors = plt.cm.get_cmap("Greys",len(np.unique(z_normalized.round(decimals=4)))*2)
    i=0
    for index,row in tqdm(distance_df.iterrows(), total=distance_df.shape[0]):
        x,y = row["x_center"], row["y_center"]
        coords = np.array([[x-tile_size,y-tile_size],[x-tile_size,y+tile_size],[x+tile_size,y+tile_size],[x+tile_size,y-tile_size]])
        poly:Polygon = Polygon(coords)
        patch1 = PolygonPatch(poly, fc=mpl.colors.rgb2hex(colors(z_normalized[index])),ec=mpl.colors.rgb2hex(colors(z_normalized[index])), alpha=1, zorder=1)
        ax.add_patch(patch1)

        patch2 = PolygonPatch(poly, fc=mpl.colors.rgb2hex(colors(z_2_normalized[index])),ec=mpl.colors.rgb2hex(colors(z_2_normalized[index])), alpha=1, zorder=1)
        ax_2.add_patch(patch2)

        patch3 = PolygonPatch(poly, fc=mpl.colors.rgb2hex(colors(z_3_normalized[index])),ec=mpl.colors.rgb2hex(colors(z_3_normalized[index])), alpha=1, zorder=1)
        ax_3.add_patch(patch3)

        i+=1
        #if(i>10000):
        #    break



    ax.autoscale(enable=True, axis="both", tight=None)
    ax_2.autoscale(enable=True, axis="both", tight=None)
    ax_3.autoscale(enable=True, axis="both", tight=None)
    figure.savefig(rospack.get_path('usv_map')+"/scripts/voroni_field.pdf")
    figure_2.savefig(rospack.get_path('usv_map')+"/scripts/distance_obstacle.pdf")
    figure_3.savefig(rospack.get_path('usv_map')+"/scripts/distance_voronoi.pdf")
    #plt.show()

main()