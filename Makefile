CGV_DIR := $(CURDIR)/cgv
NAME := ellipsoid-trajectory-vis

DIST = $(CURDIR)/dist/$(NAME)
GZIP = $(CURDIR)/dist/$(NAME).tar.gz

.PHONY: dist release cgv

# Create a some what *portable* distribution of the visualization tool. The core idea is to install
# the CGV framework and the plugin in a local *distribition directory* togther with a custom launch
# script that adjusts the link path. The distribution directory can be transfered to other Debian-
# based Systems without the need of compiling the CGV framework and plugin.
dist: release cgv
	chrpath --delete $(DIST)/bin/cgv_viewer
	cp dist/launch.sh $(DIST)/$(NAME)

	find $(DIST)/bin/ -type f -not -name cgv_viewer -delete
	rm -r $(DIST)/cmake
	rm -r $(DIST)/include

gzip: dist
	tar -C dist/ -czf $(GZIP) $(NAME)

upload: gzip
	scp $(GZIP) square-src.de:~/domains/franzi.square-src.de/

# Build release of ellipsoid-trajectory plugin and install it in dist directory
release: cgv
	mkdir -p dist/build/ellipsoid-trajectory-vis
	cd dist/build/ellipsoid-trajectory-vis; \
		cmake -D CMAKE_BUILD_TYPE=Release \
			  -D CMAKE_PREFIX_PATH=$(CURDIR)/dist/build/cgv \
			  -D CMAKE_INSTALL_PREFIX=$(DIST) $(CURDIR); \
		make -j install

# Build release of CGV framework and install it in dist directory
cgv:
	mkdir -p dist/build/cgv
	cd dist/build/cgv; \
		cmake -D CMAKE_BUILD_TYPE=Release \
			  -D CMAKE_INSTALL_PREFIX=$(DIST) $(CURDIR)/cgv; \
		make -j install

clean:
	rm -rf dist/build
	rm -rf $(DIST)
	rm -rf $(GZIP)