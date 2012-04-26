PROJECT( interest_detect_proj )
FIND_PACKAGE( OpenCV REQUIRED )
ADD_EXECUTABLE( interest-detect interest-detect.cpp )
TARGET_LINK_LIBRARIES( interest-detect ${OpenCV_LIBS} )
