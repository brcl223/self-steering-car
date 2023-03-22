#!/bin/bash
# Example: ./run.sh -p path/to/pngs -f 30 -o path/to/output/vid-123

# getops x:y:z: => ./script -x xopt -y yopt -z zopt
# while getopts u:a:f: flag
# do
#     case "${flag}" in
#         u) username=${OPTARG};;
#         a) age=${OPTARG};;
#         f) fullname=${OPTARG};;
#     esac
# done
# echo "Username: $username";
# echo "Age: $age";
# echo "Full Name: $fullname";

while getopts o:f:p: flag
do
    case "${flag}" in
        o) OUTNAME=${OPTARG};;
        f) FRAMERATE=${OPTARG};;
        p) PATH=${OPTARG};;
    esac
done

# ffmpeg -framerate 30 -pattern_type glob -i '/path/to/*.png' -c:v libx264 -pix_fmt yuv420 out.mp4
/opt/homebrew/bin/ffmpeg -framerate ${FRAMERATE} -pattern_type glob -i "${PATH}/*.png" -c:v libx264 -pix_fmt yuv420p "${OUTNAME}.mp4"
