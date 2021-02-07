import os, os.path
import numpy as np
import glob
import open3d as o3d
from pathlib import Path

def findInArray(arr, line):
    for entry in arr:
        if entry.find(line[:-11]) != -1:
            return 1
    return 0

correct_path = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/RandLaNet/evaluation/labeled/"
dir_path = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/raw_pointcloud_data/cbrg_output/"
#aufnahme = "1_"

for y in range(20):
    y+=1
    for k in range(3):
        k+=1
        nummer = str(y) + "_"  + str(k)
        print("Aufnahme {}".format(nummer))
        correct_file = open(correct_path + nummer + ".pcd")
        correct_pcd = o3d.io.read_point_cloud(correct_path + nummer + ".pcd")
        correct_xyz = np.asarray(correct_pcd.points)
        correct = correct_file.read().split("\n")

        #remove first 10
        for i in range(10):
            correct.pop(0)

        dic = {
            "apple": [],
            "banana": [],
            "bread": [],
            "egg": [],
            "glass_drink": [],
            "cucumber": [],
            "yoghurt": [],
            "cheese": [],
            "cereals": [],
            "cup_drink": [],
            "tomato": [],
            "sausage": [],
        }

        for line in correct:
            if line.find(" 1 ") != -1:
                dic["apple"].append(line[:-13])
            elif line.find(" 2 ") != -1:
                dic["banana"].append(line[:-13])
            elif line.find(" 3 ") != -1:
                dic["bread"].append(line[:-13])
            elif line.find(" 4 ") != -1:
                dic["egg"].append(line[:-13])
            elif line.find(" 5 ") != -1:
                dic["glass_drink"].append(line[:-13])
            elif line.find(" 6 ") != -1:
                dic["cucumber"].append(line[:-13])
            elif line.find(" 7 ") != -1:
                dic["yoghurt"].append(line[:-13])
            elif line.find(" 8 ") != -1:
                dic["cheese"].append(line[:-13])
            elif line.find(" 9 ") != -1:
                dic["cereals"].append(line[:-13])
            elif line.find(" 10 ") != -1:
                dic["cup_drink"].append(line[:-13])
            elif line.find(" 11 ") != -1:
                dic["tomato"].append(line[:-13])
            elif line.find(" 12 ") != -1:
                dic["sausage"].append(line[:-13])


        print("############### correct label count ###############")
        print("apfel: {}".format(len(dic["apple"])))
        print("banane: {}".format(len(dic["banana"])))
        print("brot: {}".format(len(dic["bread"])))
        print("ei: {}".format(len(dic["egg"])))
        print("glass: {}".format(len(dic["glass_drink"])))
        print("gurke: {}".format(len(dic["cucumber"])))
        print("joghurt: {}".format(len(dic["yoghurt"])))
        print("kaese: {}".format(len(dic["cheese"])))
        print("muesli: {}".format(len(dic["cereals"])))
        print("tasse: {}".format(len(dic["cup_drink"])))
        print("tomate: {}".format(len(dic["tomato"])))
        print("wurst: {}".format(len(dic["sausage"])))
        print("###################################################")
        print(" ")

        log = open(dir_path + "log_2.txt", "a")

        log.write("####################### "+ nummer +" ####################### \n")
        log.write(" \n")
        log.write("############### correct label count ###############\n")
        log.write("apfel: {} \n".format(len(dic["apple"])))
        log.write("banane: {}\n".format(len(dic["banana"])))
        log.write("brot: {}\n".format(len(dic["bread"])))
        log.write("ei: {}\n".format(len(dic["egg"])))
        log.write("glass: {}\n".format(len(dic["glass_drink"])))
        log.write("gurke: {}\n".format(len(dic["cucumber"])))
        log.write("joghurt: {}\n".format(len(dic["yoghurt"])))
        log.write("kaese: {}\n".format(len(dic["cheese"])))
        log.write("muesli: {}\n".format(len(dic["cereals"])))
        log.write("tasse: {}\n".format(len(dic["cup_drink"])))
        log.write("tomate: {}\n".format(len(dic["tomato"])))
        log.write("wurst: {}\n".format(len(dic["sausage"])))
        log.write("###################################################\n")
        log.write(" \n")



        for cbrg_path in glob.iglob(dir_path+nummer+'*.pcd'):
            cbrg_output = open(cbrg_path)
            path_length = len(dir_path) + len(nummer) +1
            lebensmittel = cbrg_path[path_length:-4]

            cbrg_pcd = o3d.io.read_point_cloud(cbrg_path)
            cbrg = cbrg_output.read().split("\n")

            #o3d.visualization.draw_geometries([cbrg_pcd])

            for i in range(11):
                cbrg.pop(0)

            cbrg.pop(len(cbrg) - 1)
            #print(cbrg)

            counter = 0

            for l in cbrg:
                counter += findInArray(dic[lebensmittel], l)

            
            print(" ")
            print("{}: {} / {} = {}".format(lebensmittel, counter, len(dic[lebensmittel]), round(counter/len(dic[lebensmittel]),5))) 
            log.write("{}: {} / {} = {}\n".format(lebensmittel, counter, len(dic[lebensmittel]), round(counter/len(dic[lebensmittel]),5))) 
            print(" ")

        dic.clear()
        log.write(" \n")
        log.close()