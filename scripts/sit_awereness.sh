#!/bin/bash

mplayer tv:// -tv driver=v4l2:device=/dev/video6:width=640:height=360:fps=30:outfmt=yuy2
