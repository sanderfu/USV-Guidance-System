
import rospy
import rosbag
import rospkg
from usv_msgs.msg import Colav,ColavPath,ColavPathElement
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#Thank me later
#http://wiki.ros.org/rosbag/Cookbook

def main():
    rospack = rospkg.RosPack()
    base_path:str = rospack.get_path('usv_realtime_recorder')+"/data/missions/"
    #mission_name:str = "headon_colav_debug_normal_0/"
    mission_name:str = "headon_colav_debug_bugfix_0/"
    print("File path: ",base_path+mission_name+"data.bag")

    figure,ax = plt.subplots(2,1)


    bag = rosbag.Bag(base_path+mission_name+"data.bag")
    plotpause=0.00001
    for topic, msg, t in bag.read_messages(topics=['/Viknes830/colav/debug']):
        typed_msg:Colav = msg
        for option_index,path_option in enumerate(typed_msg.path_options):
            path_array = np.empty((0,3))
            typed_path_option:ColavPath = path_option
            for element in typed_path_option.path:
                #Iterating path options
                typed_element:ColavPathElement = element
                #print("Lon: ", typed_element.lon," Lat: ", typed_element.lat," Course: ", typed_element.course)
                path_array = np.append(path_array,np.array([[typed_element.lon,typed_element.lat,typed_element.course]]),axis=0)
            #print(path_array)
            ax[0].plot(path_array[:,0],path_array[:,1],label=str(option_index))
            #Line numbering
            #ax[0].text(path_array[-110,0], path_array[-110,1], str(option_index), fontsize=12)
            #ax[1].plot(path_array[:-110,2]*180/np.pi,label=str(option_index))
            #ax[1].text(path_array[:,2].size, path_array[-110,2]*180/np.pi, str(option_index), fontsize=12)  
            

        figure.canvas.draw_idle()
        plt.show(block=False)
        ax[0].legend()
        ax[1].legend()
        input("Hit any key on terminal to continue")
        ax[0].cla()
        ax[1].cla()
            
            
    ax[0].legend()
    ax[1].legend()
    plt.show()
    bag.close()



main()