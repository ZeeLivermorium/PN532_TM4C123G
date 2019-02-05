#*****************************************************
# @brief:  script for cleaning up files
# @author: Zee Lv
# @date:   Jan 22, 2019
#************************

# clean up all build product in all sub folders
find . -name "*.o" -exec rm -rf {} \;
find . -name "*.d" -exec rm -rf {} \;
find . -name "build" -type d | xargs rm -rf

# clean up keil build product in all subfolders
find . -name "*.dep" -exec rm -rf {} \;
find . -name "*.bak" -exec rm -rf {} \;
find . -name "*.htm" -exec rm -rf {} \;
find . -name "*.axf" -exec rm -rf {} \;
find . -name "*.crf" -exec rm -rf {} \;
find . -name "*.lnp" -exec rm -rf {} \;
find . -name "*.map" -exec rm -rf {} \;
find . -name "*.uvopt" -exec rm -rf {} \;
find . -name "*.uvgui.*" -exec rm -rf {} \;
find . -name "*.lst" -exec rm -rf {} \;
