CC ?= gcc
CFLAGS := $(filter-out -fcanon-prefix-map,$(CFLAGS))

# Use pkg-config if available, otherwise fall back to sysroot-relative paths
PKG_CONFIG ?= pkg-config
ifneq ($(shell which $(PKG_CONFIG) 2>/dev/null),)
  DRM_CFLAGS := $(shell $(PKG_CONFIG) --cflags libdrm)
  DRM_LIBS := $(shell $(PKG_CONFIG) --libs libdrm)
else
  # -I =/path uses the GCC sysroot-relative lookup (--sysroot prefix is prepended)
  DRM_CFLAGS := -I =/usr/include/libdrm
  DRM_LIBS := -ldrm
endif

CFLAGS += -Wall $(DRM_CFLAGS)
LIBS += $(DRM_LIBS)

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: cam2display

cam2display: drm_display.o v4l2_capture.o cam2display.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o cam2display
