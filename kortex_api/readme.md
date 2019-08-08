<!-- 
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed 
* under the terms of the BSD 3-Clause license. 
*
* Refer to the LICENSE file for details.
*
* -->

# Kortex API
This package contains all the C++ files used by the driver node. This package is mandatory because most of the other packages use it. More detailed documentation is available on the Kortex API [repository](https://github.com/Kinovarobotics/kortex).

## Downloading the Kortex API

When you use `catkin_make` to build `ros_kortex`, the [download_kortex_api](./scripts/download_kortex_api.bash) script automatically checks whether or not the API was downloaded. If it is not present, the script downloads and extracts it in the correct folders.

You can also manually download the API, extract it and copy the contents of the include and lib folders: 
 - ```cpp/linux_gcc_x86-64/lib``` to the ```kortex_api/lib``` folder
 - ```cpp/linux_gcc_x86-64/include``` to the ```kortex_api/include``` folder
 
