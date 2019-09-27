LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
#LOCAL_CFLAGS +=
LOCAL_C_INCLUDES += system/core/include/cutils/ external/speex/include/ $(LOCAL_PATH)/webrtc
LOCAL_SRC_FILES:= $(call all-c-files-under)
LOCAL_SRC_FILES+= $(call all-cpp-files-under)
LOCAL_STATIC_LIBRARIES += libspeex 
LOCAL_SHARED_LIBRARIES := libcutils libutils libmedia liblog
LOCAL_LDFLAGS := -L$(LOCAL_PATH) -lTTUpdate
LOCAL_MODULE := satfi
LOCAL_MULTILIB := 32
#LOCAL_MODULE_PATH := $(LOCAL_PATH)/bin
include $(BUILD_EXECUTABLE)
