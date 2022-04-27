#To remove error with typing of list[object]
from __future__ import annotations

from dataclasses import dataclass
from email import message
import rospy
import rosbag
import rospkg
from usv_msgs.msg import Colav,ColavPath,ColavPathElement
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.collections  as mc
import matplotlib as mpl
import numpy as np
from osgeo import ogr, osr, gdal
from descartes import PolygonPatch
from shapely.geometry import Polygon
from shapely.wkt import loads

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


#Thank me later
#http://wiki.ros.org/rosbag/Cookbook


#Class object for the COLAV Debug message (immutable)
class COLAVDebug:
    time:float
    speed_correction:float
    course_correction:float
    cost:float
    path: np.ndarray
    path_options:list[np.ndarray] = []
    cost_options:np.ndarray
    def __init__(self):
        #For some reason, if I dont have this all containers just inherit from the state of the last container (WTF Python?!)
        self.path_options = []
        self.cost_options = np.empty((0,))


def main():
    rospack = rospkg.RosPack()
    base_path:str = rospack.get_path('usv_realtime_recorder')+"/data/missions/"
    mission_name:str = "colav_crossing_right_0/"
    map_name:str = "outside_ny_max_300"
    print("File path: ",base_path+mission_name+"data.bag")

    figure,ax = plt.subplots(1,1)
    
    ##Plot background
    datasource_path = rospack.get_path('usv_map')+"/data/mission_regions/"+map_name+"/region.sqlite"
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
            patch1 = PolygonPatch(poly, fc=GREEN, ec=GREEN, alpha=1, zorder=2)
            ax.add_patch(patch1)

    bag = rosbag.Bag(base_path+mission_name+"data.bag")

    message_list:list[COLAVDebug] = []
    for topic, msg, t in bag.read_messages(topics=['/Viknes830/colav/debug']):
        container:COLAVDebug = COLAVDebug()
        typed_msg:Colav = msg
        container.time = typed_msg.time
        container.speed_correction = typed_msg.speed_correction
        container.course_correction = typed_msg.course_correction
        container.cost = typed_msg.cost
        choosen_path_array = np.empty((0,3))
        for element in typed_msg.path.path:
            choosen_path_array = np.append(choosen_path_array,np.array([[typed_element.lon,typed_element.lat,typed_element.course]]),axis=0)
        container.path = choosen_path_array

        #Determining cost options and path options
        cost_options_list = []
        for option_index,(path_option,cost_option) in enumerate(zip(typed_msg.path_options,typed_msg.cost_options)):
            
            path_array = np.empty((0,3))
            typed_path_option:ColavPath = path_option
            for element in typed_path_option.path:
                #Iterating path options
                typed_element:ColavPathElement = element
                path_array = np.append(path_array,np.array([[typed_element.lon,typed_element.lat,typed_element.course]]),axis=0)
            container.path_options.append(path_array)
            cost_options_list.append(cost_option)
        container.cost_options = np.array(cost_options_list)
        message_list.append(container)


    cost_option_max_clipping_mask = 25
    counter = 0
    for message in message_list:
        #Clip cost options due to presence of huge values occasionally (TODO: Chedk why these huge values occur)
        np.clip(message.cost_options,0,cost_option_max_clipping_mask,out=message.cost_options)

        #Normalize cost values to lie in range(0,1)
        cost_options_normalized = (message.cost_options-min(message.cost_options))/(max(message.cost_options)-min(message.cost_options))
        colors = plt.cm.get_cmap("cool",len(np.unique(cost_options_normalized.round(decimals=4)))*2)
        color_matrix = np.eye(4)

        colors_array=[]
        lines_array=[]
        
        path_options_numpy = np.array(message.path_options,dtype=object)
        inds = message.cost_options.argsort()
        sorted_options = path_options_numpy[inds]
        sorted_costs_normalized = cost_options_normalized[inds]
        sorted_costs = message.cost_options[inds]
        for option_index,path_option in enumerate(sorted_options):
            color_array = np.array(colors(sorted_costs_normalized[option_index]))
            alpha_matrix = np.eye(4)
            segment_count = len(path_option[:,0])
            alpha_grade = np.flip(np.linspace(0,1,segment_count)**2)
            for i in range(segment_count-1):
                alpha_matrix[3,3] = alpha_grade[i]*0.75
                colors_array.append(alpha_matrix@color_array)
                line = [(path_option[i,0],path_option[i,1]),(path_option[i+1,0],path_option[i+1,1])]
                lines_array.append(line)
            #ax.text(path_option[-1,0],path_option[-1,1],s=str(sorted_costs_normalized[option_index]))
        #Reverse lists for visualization
        lines_array.reverse()
        colors_array.reverse()
        lc = mc.LineCollection(lines_array,colors=colors_array,linewidth=2.5,zorder=2)
        ax.add_collection(lc)

        #Highlight choosen option
        #segment_count = len(sorted_options[0])
        #choosen_path_segments = []
        #for i in range(segment_count-1):
        #    alpha_matrix[3,3] = alpha_grade[i]*0.25
        #    colors_array.append(alpha_matrix@color_array)
        #    line = [(sorted_options[0][i,0],sorted_options[0][i,1]),(sorted_options[0][i+1,0],sorted_options[0][i+1,1])]
        #    choosen_path_segments.append(line)
        #choosen_path_lc = mc.LineCollection(choosen_path_segments,linewidth=20,zorder=1)
        #ax.add_collection(choosen_path_lc)

        #if counter>=235:
        #    figure.canvas.draw_idle()
        #    plt.autoscale(enable=True, axis="both", tight=None)
        #    plt.show(block=False)
        #    plt.waitforbuttonpress()
        #ax.cla()
        #print("Message: ", counter)
        #counter+=1



            
    ax.legend()
    plt.autoscale(enable=True, axis="both", tight=None)
    plt.show()
    bag.close()



main()