# simple status viewer and config changer for the aircraft
import threading

import dearpygui.dearpygui as dpg
import requests
import time

dpg.create_context()
dpg.create_viewport(title='Custom Title', width=600, height=600)
dpg.setup_dearpygui()


def getData():
    threading.Timer(0.05, getData).start()
    try:
        response = requests.get("http://192.168.4.1/json", timeout=0.1)
        data = response.json()
        pressure = data['sensordata'][0]["pressure"]
        accels = [data['sensordata'][2]['xaccel'],
                  10-data['sensordata'][2]['zaccel'],
                  data['sensordata'][2]['yaccel']]
        dpg.set_value('accelGraph', accels)
        print(accels)
    except Exception as e:
        print(e)


with dpg.window(tag="Primary Window"):

    dpg.add_3d_slider(min_x=-10, min_y=-10, min_z=-10, max_x=10,
                      max_y=10, max_z=10, width=25, height=25, tag="accelGraph")

    getData()
dpg.show_viewport()
dpg.set_primary_window("Primary Window", True)
dpg.start_dearpygui()
dpg.destroy_context()
