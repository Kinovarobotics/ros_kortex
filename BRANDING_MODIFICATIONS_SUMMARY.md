# KINOVA KORTEX™ Branding Modifications Summary

This document summarizes all modifications made to replace "kortex" with "KINOVA KORTEX™" in documentation and prose contexts only.

## Branch Information
- **Branch Name**: `kinova-kortex-tm`
- **Base Branch**: `humble`

## Modification Rules Applied
- ✅ Target only: `*.md`, `*.rst`, `*.txt`, and `package.xml` description fields
- ✅ Case-insensitive match (kortex, Kortex, KORTEX → KINOVA KORTEX™)
- ❌ Do NOT change: identifiers, package names, repo names (e.g., ros2_kortex)
- ❌ Do NOT change: URLs, links, image paths
- ❌ Do NOT change: filenames or folder names
- ❌ Skip fenced code blocks (```...```)
- ✅ Keep UTF-8 encoding so ™ is preserved

## Files Modified (12 total)

### 1. README.md
**Changes:**
```diff
- # ROS 2 Kortex
+ # ROS 2 KINOVA KORTEX™

- > Kinova® Kortex™ is the common software platform
+ > Kinova® KINOVA KORTEX™ is the common software platform

- ROS2 Kortex is the official ROS2 package to interact with Kortex and its related products. It is built upon the Kortex API
+ ROS2 KINOVA KORTEX™ is the official ROS2 package to interact with KINOVA KORTEX™ and its related products. It is built upon the KINOVA KORTEX™ API

- 1. build everything except KINOVA KORTEX™ related packages
+ 1. build everything except kortex related packages
```

### 2. kortex_description/readme.md
**Changes:**
```diff
- # Kortex Description
+ # KINOVA KORTEX™ Description

- This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots.
+ This package contains the URDF (Unified Robot Description Format), STL and configuration files for the KINOVA KORTEX™-compatible robots.

- NOTE: deprecated according to the [Kortex repo]
+ NOTE: deprecated according to the [KINOVA KORTEX™ repo]
```

### 3. kortex_driver/README.md
**Changes:**
```diff
- # ROS 2 Kortex Driver
+ # ROS 2 KINOVA KORTEX™ Driver

- The ROS 2 Kortex driver implements the `ros2_control` hardware interface
+ The ROS 2 KINOVA KORTEX™ driver implements the `ros2_control` hardware interface
```

### 4. kortex_moveit_config/readme.md
**Changes:**
```diff
- # Kortex MoveIt 2 Config
+ # KINOVA KORTEX™ MoveIt 2 Config

- See the `kortex_description/arms` folder for a list of supported Kinova Kortex robots.
+ See the `kortex_description/arms` folder for a list of supported Kinova KINOVA KORTEX™ robots.

- See the `kortex_description/grippers` folder for a list of supported Kinova Kortex grippers.
+ See the `kortex_description/grippers` folder for a list of supported Kinova KINOVA KORTEX™ grippers.
```

### 5. kortex_description/CHANGELOG.rst
**Changes:**
```diff
- * Combined launch file for kortex\_(moveit/control)
+ * Combined launch file for KINOVA KORTEX™\_(moveit/control)
```

### 6. kortex_api/package.xml
**Changes:**
```diff
- <description>kortex_api</description>
+ <description>KINOVA KORTEX™ API</description>
```

### 7. kortex_bringup/package.xml
**Changes:**
```diff
- <description>Launch file and run-time configurations, e.g. controllers.</description>
+ <description>Launch file and run-time configurations for KINOVA KORTEX™, e.g. controllers.</description>
```

### 8. kortex_description/package.xml
**Changes:**
```diff
- <p>URDF and xacro description package for Kortex robots</p>
+ <p>URDF and xacro description package for KINOVA KORTEX™ robots</p>

- <p>This package contains configuration data, 3D models and launch files for Kortex arms and supported grippers</p>
+ <p>This package contains configuration data, 3D models and launch files for KINOVA KORTEX™ arms and supported grippers</p>
```

### 9. kortex_driver/package.xml
**Changes:**
```diff
- <description>ROS2 driver package for the Kinova Robot Hardware.</description>
+ <description>ROS2 driver package for the Kinova KINOVA KORTEX™ Robot Hardware.</description>
```

### 10. kortex_moveit_config/kinova_gen3_6dof_robotiq_2f_85_moveit_config/package.xml
**Changes:**
```diff
- An automatically generated package with all the configuration and launch files for using the gen3 with the MoveIt Motion Planning Framework
+ An automatically generated package with all the configuration and launch files for using the KINOVA KORTEX™ gen3 with the MoveIt Motion Planning Framework
```

### 11. kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/package.xml
**Changes:**
```diff
- An automatically generated package with all the configuration and launch files for using the gen3 with the MoveIt Motion Planning Framework
+ An automatically generated package with all the configuration and launch files for using the KINOVA KORTEX™ gen3 with the MoveIt Motion Planning Framework
```

### 12. kortex_moveit_config/kinova_gen3_lite_moveit_config/package.xml
**Changes:**
```diff
- An automatically generated package with all the configuration and launch files for using the gen3_lite with the MoveIt Motion Planning Framework
+ An automatically generated package with all the configuration and launch files for using the KINOVA KORTEX™ gen3_lite with the MoveIt Motion Planning Framework
```

## Files Explicitly NOT Modified
- All code files (`.py`, `.cpp`, `.h`, `.c`, etc.)
- All configuration files (`.yaml`, `.yml`, `.launch.py`, etc.)
- All URL references and links
- Package names and identifiers (ros2_kortex, kortex_, etc.)
- License files
- Repository configuration files (.repos, etc.)

## Verification Steps
1. Confirmed no URLs were modified
2. Confirmed no package names/identifiers were changed
3. Confirmed code blocks were skipped
4. Verified UTF-8 encoding preserved for ™ symbol
5. Checked that build/install commands remain functional

## Git Commands Used
```bash
# Create and switch to branch
git checkout -b kinova-kortex-tm

# Verify changes
git diff --stat
git diff --name-only
```

## Replication Instructions
1. Create branch: `git checkout -b kinova-kortex-tm`
2. Apply the exact text replacements shown above to each file
3. Verify no code identifiers, URLs, or package names were affected
4. Test that the ™ symbol displays correctly in UTF-8
5. Ensure all build/install commands still function properly

**Total Files Modified**: 12
**Total Replacements**: ~24 occurrences of "kortex" → "KINOVA KORTEX™"
