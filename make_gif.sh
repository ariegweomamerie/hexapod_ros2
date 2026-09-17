#!/usr/bin/env bash
# Convert a video (e.g. a Gazebo screen recording) into a README-ready GIF.
# Usage:  ./make_gif.sh <input_video> [output.gif] [fps] [width]
# Example: ./make_gif.sh my_walk.mp4 docs/media/hexapod_walking.gif 12 640
#
# Uses the ffmpeg that ships in the ffmpeg snap (no system ffmpeg here). If you
# install a normal ffmpeg (sudo apt install ffmpeg), just replace $FF with ffmpeg.
set -e
IN="${1:?usage: make_gif.sh <input_video> [output.gif] [fps] [width]}"
OUT="${2:-docs/media/hexapod_walking.gif}"
FPS="${3:-12}"
WIDTH="${4:-640}"

SNAP=/snap/ffmpeg-2404/156
FF="$SNAP/usr/bin/ffmpeg"
export LD_LIBRARY_PATH="$SNAP/usr/lib/x86_64-linux-gnu"

mkdir -p "$(dirname "$OUT")"
PALETTE="$(mktemp --suffix=.png)"
# two-pass palette for clean colours + small size
"$FF" -y -i "$IN" -vf "fps=$FPS,scale=$WIDTH:-1:flags=lanczos,palettegen" "$PALETTE"
"$FF" -y -i "$IN" -i "$PALETTE" \
      -lavfi "fps=$FPS,scale=$WIDTH:-1:flags=lanczos [x]; [x][1:v] paletteuse" "$OUT"
rm -f "$PALETTE"
echo "wrote $OUT"
