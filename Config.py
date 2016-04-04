# -*- coding: utf-8 -*-


class Config():

    def __init__(self):

        self.isFake = True
        self.isLoginRequired = False
        self.FENCE_PIN = 12
        self.INSTALL_PATH = "/opt/spinitalia/service/"
        self.SIM_PATH = "/tmp/spinitalia/simulation/"
        self.LOG_PATH = "/tmp/spinitalia/logs/"
        #self.SIM_PATH = "/mnt/nas/media/"
        self.DEF_MOVE = "/opt/spinitalia/default_position/"
        self.MOT_DATA = "/tmp/spinitalia/motor_data/"
        self.MD5SUM_EXEC = '/usr/bin/md5sum'
        if self.isFake:
            self.MOT_EXT = ".mot.fake"
        else:
            self.MOT_EXT = ".mot"
        if self.isFake:
            self.POS_PIPE = "/tmp/fake_alma_3d_spinitalia_pos_stream_pipe"
        else:
            self.POS_PIPE = "/tmp/alma_3d_spinitalia_pos_stream_pipe"
