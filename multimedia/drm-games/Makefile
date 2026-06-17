CC ?= gcc
CFLAGS := $(filter-out -fcanon-prefix-map,$(CFLAGS))

# Use pkg-config if available, otherwise fall back to sysroot-relative paths
PKG_CONFIG ?= pkg-config
ifneq ($(shell which $(PKG_CONFIG) 2>/dev/null),)
  DRM_CFLAGS := $(shell $(PKG_CONFIG) --cflags libdrm)
  DRM_LIBS := $(shell $(PKG_CONFIG) --libs libdrm) -lm
else
  # -I =/path uses the GCC sysroot-relative lookup (--sysroot prefix is prepended)
  DRM_CFLAGS := -I =/usr/include/libdrm
  DRM_LIBS := -ldrm -lm
endif

CFLAGS += -Wall $(DRM_CFLAGS)
LIBS += $(DRM_LIBS)

GAMES := drm-game drm-snake drm-tetris drm-invaders drm-flappy \
         drm-racing drm-platformer drm-asteroids drm-puzzle drm-pong \
         drm-catch drm-shooter drm-breaker drm-ninja drm-tower drm-whack

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: $(GAMES)

$(GAMES): %: %.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o $(GAMES)

.PHONY: all clean
