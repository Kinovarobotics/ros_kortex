from conans import ConanFile, CMake
import os
import shutil
import time

class ROSKortexConan(ConanFile):  

    def requirements(self):
        self.requires("kortex_api_cpp/2.3.0-r.34@kortex/stable")
