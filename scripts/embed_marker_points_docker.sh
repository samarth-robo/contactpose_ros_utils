#!/usr/bin/env bash
docker run --rm -it --mount type=bind,src=`pwd`,dst=/code --mount type=bind,src=`pwd`/data/contactdb_data,dst=/data contactpose/pymesh

# in Docker, run
# $ cd /code
# $ python scripts/embed_marker_points.py --input /data/stl_files_mm/airplane.stl --output /data/stl_files_notched
