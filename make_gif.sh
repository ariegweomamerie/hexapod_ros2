#!/usr/bin/env bash
# Convert a video (e.g. a Gazebo screen recording) or a numbered image sequence
# into a README-ready GIF.
# Usage:  ./make_gif.sh <input_video | frames/%05d.png> [output.gif] [fps] [width]
# Example: ./make_gif.sh my_walk.mp4 docs/media/hexapod_walking.gif 12 640
#
# For an image sequence (input contains %), the frames are played at [fps].
# Uses the ffmpeg that ships in the ffmpeg snap (no system ffmpeg here). If you
# install a normal ffmpeg (sudo apt install ffmpeg), just replace $FF with ffmpeg.
set -e
IN="${1:?usage: make_gif.sh <input_video | frames/%05d.png> [output.gif] [fps] [width]}"
OUT="${2:-docs/media/hexapod_walking.gif}"
FPS="${3:-12}"
WIDTH="${4:-640}"

SNAP=/snap/ffmpeg-2404/current
FF="$SNAP/usr/bin/ffmpeg"
export LD_LIBRARY_PATH="$SNAP/usr/lib/x86_64-linux-gnu"

IN_OPTS=()
case "$IN" in *%*) IN_OPTS=(-framerate "$FPS") ;; esac

mkdir -p "$(dirname "$OUT")"
PALETTE="$(mktemp --suffix=.png)"
# two-pass palette for clean colours + small size
"$FF" -y "${IN_OPTS[@]}" -i "$IN" -vf "fps=$FPS,scale=$WIDTH:-1:flags=lanczos,palettegen" "$PALETTE"
"$FF" -y "${IN_OPTS[@]}" -i "$IN" -i "$PALETTE" \
      -lavfi "fps=$FPS,scale=$WIDTH:-1:flags=lanczos [x]; [x][1:v] paletteuse" "$OUT"
rm -f "$PALETTE"
echo "wrote $OUT"
