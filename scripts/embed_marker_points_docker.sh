#!/usr/bin/env bash
docker run --rm -it --mount type=bind,src=`pwd`,dst=/code --mount type=bind,src=`pwd`/data/contactdb_data,dst=/data contactpose/pymesh
