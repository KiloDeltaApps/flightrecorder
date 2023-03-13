# simple status viewer and config changer for the aircraft
import threading

import dearpygui.dearpygui as dpg
import requests
import time

dpg.create_context()
dpg.create_viewport(title='Custom Title', width=600, height=200)
dpg.setup_dearpygui()


def getData():

    threading.Timer(0.1, getData).start()
    try:
        response = requests.get("http://192.168.4.1/json", timeout=0.1)
        data = response.json()
        pressure = data['sensordata'][0]["pressure"]
    except:
        pressure = "404"
    dpg.set_value("pressureText", pressure)


with dpg.window(tag="Primary Window"):
    dpg.add_text("Pressure: NaN", tag="pressureText")
    dpg.add_same_line()
    dpg.add_text(" hPa")
    dpg.add_text("Temperature: ")
    dpg.add_same_line()

    dpg.add_text("NaN °C")
    getData()
dpg.show_viewport()
dpg.set_primary_window("Primary Window", True)
dpg.start_dearpygui()
dpg.destroy_context()
