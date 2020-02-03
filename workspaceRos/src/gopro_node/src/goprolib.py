#!/usr/bin/env python

import time
import socket
import sys
import urllib2

import json
import constants

import cv2


class GoPro:

    def __init__(self, camera="detect", ip_address="10.5.5.9", mac_address="AA:BB:CC:DD:EE:FF", debug=True):
        if sys.version_info[0] < 2:
            print("Run on Python v2, use goprocam api for Python v3")
            exit()
        self.ip_addr = ip_address
        self._camera = ""
        self._mac_address = mac_address
        self._debug = debug
        try:
            from getmac import get_mac_address
            self._mac_address = get_mac_address(ip=self.ip_addr)
        except ImportError:
            self._mac_address = mac_address
        if camera == "detect":
            self._camera = self.whichCam()
        elif camera == "startpair":
            self.pair()
        else:
            if camera == constants.Camera.Interface.Auth or camera == "HERO3" or camera == "HERO3+" or camera == "HERO2":
                self.power_on_auth()
                time.sleep(2)
                self._camera = constants.Camera.Interface.Auth
            else:
                self._camera = constants.Camera.Interface.GPControl
                self.power_on(self._mac_address)
                self._prepare_gpcontrol()
            print("Connected to " + self.ip_addr)



    def _prepare_gpcontrol(self):
        try:
            response_raw = self._request("gp/gpControl")
            jsondata = json.loads(response_raw)
            response = jsondata["info"]["firmware_version"]
            if "HX" in response:  # Only session cameras.
                connectedStatus = False
                while connectedStatus == False:
                    req = self._request("gp/gpControl/status")
                    json_data = json.loads(req)
                    if json_data["status"]["31"] >= 1:
                        connectedStatus = True
        except (HTTPError, URLError) as error:
            self._prepare_gpcontrol()
        except timeout:
            self._prepare_gpcontrol()

        print("Camera successfully connected!")



    def _request(self, path, param="", value="", _timeout=5, _isHTTPS=False, _context=None):
        if param != "" and value == "":
            uri = "%s%s/%s/%s" % ("https://" if _isHTTPS else "http://",
                                  self.ip_addr, path, param)
            return urllib2.urlopen(uri, timeout=_timeout, context=_context).read().decode("utf-8")
        elif param != "" and value != "":
            uri = "%s%s/%s/%s/%s" % ("https://" if _isHTTPS else "http://",
                                     self.ip_addr, path, param, value)
            return urllib2.urlopen(uri, timeout=_timeout, context=_context).read().decode("utf-8")
        elif param == "" and value == "":
            uri = "%s%s/%s" % ("https://" if _isHTTPS else "http://",
                               self.ip_addr, path)
            return urllib2.urlopen(uri, timeout=_timeout, context=_context).read().decode("utf-8")


    def whichCam(self):
        """ This returns what type of camera is currently connected.
         - gpcontrol: HERO4 Black and Silver, HERO5 Black and Session, HERO Session (formally known as HERO4 Session), HERO+ LCD, HERO+.
         - auth: HERO2 with WiFi BacPac, HERO3 Black/Silver/White, HERO3+ Black and Silver. """
        if self._camera != "":
            return self._camera
        else:
            response_raw = self._request("gp/gpControl")
            jsondata = json.loads(response_raw)
            response = jsondata["info"]["firmware_version"]
            response_parsed = 3
            exception_found = False
            if "HD" in response:
                response_parsed = response.split("HD")[1][0]
            exceptions = ["HX", "FS", "HD3.02", "H18", "H19"]
            for camera in exceptions:
                if camera in response:
                    exception_found = True
                    break
            # HD4 (Hero4), HD5 (Hero5), HD6 (Hero6)... Exceptions: HX (HeroSession), FS (Fusion), HD3.02 (Hero+), H18 (Hero 2018)
            if int(response_parsed) > 3 or exception_found:
                print(jsondata["info"]["model_name"] +
                      "\n" + jsondata["info"]["firmware_version"])
                self._prepare_gpcontrol()
                self._camera = constants.Camera.Interface.GPControl
            else:
                response = self._request("camera/cv")
                if b"Hero3" in response:  # should detect HERO3/3+
                    self._camera = constants.Camera.Interface.Auth

            return self._camera


    def gpControlExecute(self, param):
        """sends Parameter to gpControl/execute"""
        return self._request("gp/gpControl/execute?" + param)


    def livestream(self, option):
        """start livestreaming
        option = "start"/"stop"
        """
        if option == "start":
            if self.whichCam() == constants.Camera.Interface.GPControl:
                return self.gpControlExecute(
                    "p1=gpStream&c1=restart")
            else:
                return self.sendCamera("PV", "02")
        if option == "stop":
            if self.whichCam() == constants.Camera.Interface.GPControl:
                return self.gpControlExecute("p1=gpStream&c1=stop")
            else:
                return self.sendCamera("PV", "00")




    def video_settings(self, res, fps="none"):
        """Change video resolution and FPS
        See constants.Video.Resolution"""
        if self.whichCam() == constants.Camera.Interface.GPControl:
            if fps != "none":
                x = "constants.Video.FrameRate.FR" + fps
                videoFps = eval(x)
                return self.gpControlSet(constants.Video.FRAME_RATE, videoFps)
            x = "constants.Video.Resolution.R" + res
            videoRes = eval(x)
            return self.gpControlSet(constants.Video.RESOLUTION, videoRes)
        elif self.whichCam() == constants.Camera.Interface.Auth:
            if res == "4k":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "06")
            elif res == "4K_Widescreen":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "08")
            elif res == "2kCin":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "07")
            elif res == "2_7k":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "05")
            elif res == "1440p":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "04")
            elif res == "1080p":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "03")
            elif res == "960p":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "02")
            elif res == "720p":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "01")
            elif res == "480p":
                return self.sendCamera(
                    constants.Hero3Commands.VIDEO_RESOLUTION, "00")
            if fps != "none":
                x = "constants.Hero3Commands.FrameRate.FPS" + fps
                videoFps = eval(x)
                return self.sendCamera(constants.Hero3Commands.FRAME_RATE, videoFps)


    def gpControlSet(self, param, value):
        """sends Parameter and value to gpControl/setting"""
        try:
            return self._request("gp/gpControl/setting", param, value)
        except (urllib2.HTTPError, urllib2.URLError) as error:
            return error
        except timeout:
            return error

if __name__ == '__main__':

    WRITE = False
    gpCam = GoPro()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t=time.time()
    gpCam.livestream("start")
    gpCam.video_settings(res='1080p', fps='30')
    gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
    cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
    counter = 0

    while True:
        nmat, frame = cap.read()
        cv2.imshow("GoPro OpenCV", frame)
        if WRITE == True:
            cv2.imwrite(str(counter)+".jpg", frame)
            counter += 1
            if counter >= 10:
                break
        if cv2.waitKey(1) & 0xFF == 27:
            break
        if time.time() - t >= 2.5:
            print("Send Keep Alive")
            sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
            t=time.time()
    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()


