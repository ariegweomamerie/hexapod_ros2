# Media for the README

Put the images/GIF the main `README.md` references here:

| File | What it should show | How to get it |
|------|--------------------|---------------|
| `hexapod_stand.png` | The robot standing in Gazebo (hero image) | Save one of your Gazebo screenshots here |
| `hexapod_iso.png` | Angled/iso view of the robot | Save a Gazebo screenshot |
| `hexapod_walking.gif` | The robot walking, turning and looking around | `python3 demo/record_walk_gif.py` (records straight from Gazebo; see `demo/README.md`) |

Notes:
- Keep images reasonably small (PNG < ~1 MB, GIF < ~8 MB) so the repo stays light.
- Raw videos (`*.mp4`) are gitignored on purpose — commit the GIF, not the video.
- `../../make_gif.sh` converts a screen recording (or numbered image frames) into a clean GIF.
