# Speedslide script

# WARNING

### This script is outdated and it is not working reliably anyway. Use SaiMoen's [SD Railgun](https://sai-moen.github.io/TMInterface-AS-SaiMoen/distributions/controller/incremental.html) instead.


## Disclaimer

The speedslide script is not able to begin a speedslide, it only maintains it. I recommend starting it when your speedslide_quality is already close to 1.0.
The result might not be perfect, especially on non-flat surfaces.
The speedslide script does only work on road and platform currently.
I can't guarantee the script works 100% correctly in every case, it is easy to make mistake in this kind of math.


## Usage

1. Run the script
2. Configure time range - execute command `sd from to`, e.g. `sd 15800 18000` . The script will modify inputs in that time range
3. Script will output nearly perfect inputs for sd in the python console after pressing Validate on a replay

